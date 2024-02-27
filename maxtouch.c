#include <cstdint>
#include <cstdbool>
#include "maxtouch.h"

#define DIVIDE_UNSIGNED_ROUND(numerator, denominator) (((numerator) + ((denominator) / 2)) / (denominator))
#define CPI_TO_SAMPLES(cpi, dist_in_mm) (DIVIDE_UNSIGNED_ROUND((cpi) * (dist_in_mm) * 10, 254))
#define SAMPLES_TO_CPI(samples, dist_in_mm) (DIVIDE_UNSIGNED_ROUND((samples) * 254, (dist_in_mm) * 10))

// By default we assume all available X and Y pins are in use, but a designer
// may decide to leave some pins unconnected, so the size can be overridden here.
#ifndef MXT_MATRIX_X_SIZE
#define MXT_MATRIX_X_SIZE information.matrix_x_size
#endif
#ifndef MXT_MATRIX_Y_SIZE
#define MXT_MATRIX_Y_SIZE information.matrix_y_size
#endif
#define MXT_DEFAULT_DPI 600
#define MXT_TOUCH_THRESHOLD 18
#define MXT_GAIN 4
#define MXT_DX_GAIN 255
#define NUM_FINGERS 5 // Can be up to 10

// Data from the object table. Registers are not at fixed addresses, they may vary between firmware
// versions. Instead must read the addresses from the object table.
static uint16_t t2_encryption_status_address = 0;
static uint16_t t5_message_processor_address = 0;
static uint16_t t5_max_message_size = 0;
static uint16_t t6_command_processor_address = 0;
static uint16_t t7_powerconfig_address = 0;
static uint16_t t8_acquisitionconfig_address = 0;
static uint16_t t44_message_count_address = 0;
static uint16_t t46_cte_config_address = 0;
static uint16_t t100_multiple_touch_touchscreen_address = 0;

// The object table also contains report_ids. These are used to identify which object generated a
// message. Again we must lookup these values rather than using hard coded values.
// Most messages are ignored, we basically just want the messages from the t100 object for now.
static uint16_t t100_first_report_id = 0;
static uint16_t t100_second_report_id = 0;
static uint16_t t100_subsequent_report_ids[NUM_FINGERS] = {};
static uint16_t t100_num_reports = 0;

// Current driver state state
static uint16_t cpi = MXT_DEFAULT_DPI;

typedef struct {
    bool  confidence;
    bool  tip;
    uint16_t x;
    uint16_t y;
} finger_t;

typedef struct {
    finger_t fingers[NUM_FINGERS];
} digitizer_t;

void read_object_table(void)
{
    mxt_information_block information = {0};

    ////////////////////////////////////////////////////////////////////////////////////////
    // First read the start of the information block to find out how many objects we have //
    ////////////////////////////////////////////////////////////////////////////////////////
    int status = I2C_Read(MXT336UD_ADDRESS, MXT_REG_INFORMATION_BLOCK, (uint8_t *)&information,
                          sizeof(mxt_information_block));
    if (status == OK)
    {
        // On Peacock the expected result is device family: 166 with 34 objects
        printf("Found MXT %d:%d, fw %d.%d with %d objects. Matrix size %dx%d\n", information.family_id, information.variant_id,
               information.version, information.build, information.num_objects, information.matrix_x_size, information.matrix_y_size);

        /////////////////////////////////////////////////////////////////////////////////////////////
        // Now read the object table to lookup the addresses and report_ids of the various objects //
        /////////////////////////////////////////////////////////////////////////////////////////////
        
        // We accumulate report_ids as we walk the object table, the first report_id is 1.
        int report_id = 1;
        uint16_t object_table_element_address = sizeof(mxt_information_block);
        for (int i = 0; i < information.num_objects; i++)
        {
            // Read the object table entries one at a time, we could read the whole lot in one go.
            mxt_object_table_element object = {};
            status = I2C_Read(MXT336UD_ADDRESS, object_table_element_address,
                              (uint8_t *)&object, sizeof(mxt_object_table_element));
            if (status == OK)
            {
                // Note: the address should be transmitted in network byte order
                const uint16_t address = (object.position_ms_byte << 8) | object.position_ls_byte;
                switch (object.type)
                {
                case 2:
                    t2_encryption_status_address = address;
                    break;
                case 5:
                    t5_message_processor_address = address;
                    t5_max_message_size = object.size_minus_one - 1;
                    break;
                case 6:
                    t6_command_processor_address = address;
                    break;
                case 7:
                    t7_powerconfig_address = address;
                    break;
                case 8:
                    t8_acquisitionconfig_address = address;
                    break;
                case 44:
                    t44_message_count_address = address;
                    break;
                case 46:
                    t46_cte_config_address = address;
                    break;
                case 100:
                    t100_multiple_touch_touchscreen_address = address;
                    t100_first_report_id = report_id;
                    t100_second_report_id = report_id + 1;
                    for (t100_num_reports = 0; t100_num_reports < NUM_FINGERS && t100_num_reports < object.report_ids_per_instance; t100_num_reports++)
                    {
                        t100_subsequent_report_ids[t100_num_reports] = report_id + 2 + t100_num_reports;
                    }
                    break;
                }
                object_table_element_address += sizeof(mxt_object_table_element);
                report_id += object.report_ids_per_instance * (object.instances_minus_one + 1);
            }
            else
            {
                printf("Failed to read object table element. Status: %d\n", status);
            }
        }
    }
    else
    {
        printf("Failed to read object table. Status: %d\n", status);
    }
}

void write_configuration(void)
{
    /////////////////////////////////////////
    // T7: Configure power saving features //
    /////////////////////////////////////////
    if (t7_powerconfig_address)
    {
        mxt_gen_powerconfig_t7 t7 = {};
        t7.idleacqint = 32;                             // The acquisition interval while in idle mode
        t7.actacqint = 10;                              // The acquisition interval while in active mode
        t7.actv2idelto = 50;                            // The timeout for transitioning from active to idle mode
        t7.cfg = T7_CFG_ACTVPIPEEN | T7_CFG_IDLEPIPEEN; // Enable pipelining in both active and idle mode

        I2C_Write(MXT336UD_ADDRESS, t7_powerconfig_address, (uint8_t *)&t7, sizeof(mxt_gen_powerconfig_t7));
    }

    ////////////////////////////////////////
    // T8: Configure capacitive acquision //
    ////////////////////////////////////////
    if (t8_acquisitionconfig_address)
    {
        mxt_gen_acquisitionconfig_t8 t8 = {};
        // Currently just use the defaults
        I2C_Write(MXT336UD_ADDRESS, t8_acquisitionconfig_address, (uint8_t *)&t8, sizeof(mxt_gen_acquisitionconfig_t8));
    }

    //////////////////////////////////////////////////////////////
    // T46: Mutural Capacitive Touch Engine (CTE) configuration //
    //////////////////////////////////////////////////////////////
    if (t46_cte_config_address)
    {
        mxt_spt_cteconfig_t46 t46 = {};
        // Currently just use the defaults
        I2C_Write(MXT336UD_ADDRESS, t46_cte_config_address, (uint8_t *)&t46, sizeof(mxt_spt_cteconfig_t46));
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////
    // T100: Touchscreen confguration - defines an area of the sensor to use as a trackpad/touchscreen. //
    //       This object generates all our interesting report messages.                                 //
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    if (t100_multiple_touch_touchscreen_address)
    {
        mxt_touch_multiscreen_t100 cfg = {};
        int status = I2C_Read(MXT336UD_ADDRESS, t100_multiple_touch_touchscreen_address,
                              (uint8_t *)&cfg, sizeof(mxt_touch_multiscreen_t100));
        cfg.ctrl = T100_CTRL_RPTEN | T100_CTRL_ENABLE; // Enable the t100 object, and enable message reporting for the t100 object.1`
                                                       // TODO: Generic handling of rotation/inversion for absolute mode?
#ifdef DIGITIZER_INVERT_X
        cfg.cfg1 = T100_CFG_SWITCHXY | T100_CFG_INVERTY; // Could also handle rotation, and axis inversion in hardware here
#else
        cfg.cfg1 = T100_CFG_SWITCHXY; // Could also handle rotation, and axis inversion in hardware here
#endif
        cfg.scraux = 0x1;                                              // AUX data: Report the number of touch events
        cfg.numtch = NUM_FINGERS;                           // The number of touch reports we want to receive (upto 10)
        cfg.xsize = information.matrix_x_size;                         // Make configurable as this depends on the sensor design.
        cfg.ysize = information.matrix_y_size;                         // Make configurable as this depends on the sensor design.
        cfg.xpitch = MXT_SENSOR_WIDTH_MM / information.matrix_x_size;  // Pitch between X-Lines (5mm + 0.1mm * XPitch).
        cfg.ypitch = MXT_SENSOR_HEIGHT_MM / information.matrix_y_size; // Pitch between Y-Lines (5mm + 0.1mm * YPitch).
        cfg.gain = MXT_GAIN;                                           // Single transmit gain for mutual capacitance measurements
        cfg.dxgain = MXT_DX_GAIN;                                      // Dual transmit gain for mutual capacitance measurements (255 = auto calibrate)
        cfg.tchthr = MXT_TOUCH_THRESHOLD;                              // Touch threshold
        cfg.mrgthr = 5;                                                // Merge threshold
        cfg.mrghyst = 5;                                               // Merge threshold hysteresis
        cfg.movsmooth = 224;                                           // The amount of smoothing applied to movements, this tails off at higher speeds
        cfg.movfilter = 4 & 0xF;                                       // The lower 4 bits are the speed response value, higher values reduce lag, but also smoothing

        // These two fields implement a simple filter for reducing jitter, but large values cause the pointer to stick in place before moving.
        cfg.movhysti = 6; // Initial movement hysteresis
        cfg.movhystn = 4; // Next movement hysteresis

        cfg.xrange = CPI_TO_SAMPLES(cpi, MXT_SENSOR_HEIGHT_MM); // CPI handling, adjust the reported resolution
        cfg.yrange = CPI_TO_SAMPLES(cpi, MXT_SENSOR_WIDTH_MM);  // CPI handling, adjust the reported resolution

        status = I2C_Write(MXT336UD_ADDRESS, t100_multiple_touch_touchscreen_address,
                                (uint8_t *)&cfg, sizeof(mxt_touch_multiscreen_t100));
        if (status != OK)
        {
            fprintf(stderr, "T100 Configuration failed: %d\n", status);
        }
    }
}

void initialize()
{
    read_object_table();
    write_configuration();
}

// The input digitizer_report is the previous digitizer state, we return a modified state 
digitizer_t read_messages(digitizer_t digitizer_report)
{
    if (t44_message_count_address)
    {
        mxt_message_count message_count = {};

        int status = I2C_Read(MXT336UD_ADDRESS, t44_message_count_address, (uint8_t *)&message_count, sizeof(mxt_message_count));
        if (status == OK)
        {
            for (int i = 0; i < message_count.count; i++)
            {
                mxt_message message = {};
                status = I2C_Read(MXT336UD_ADDRESS, t5_message_processor_address,
                                  (uint8_t *)&message, sizeof(mxt_message));

                if (message.report_id == t100_first_report_id)
                {
                    // Unused for now, but this report contains the number of contacts
                }
                else if ((message.report_id >= t100_subsequent_report_ids[0]) &&
                         (message.report_id <= t100_subsequent_report_ids[t100_num_reports - 1]))
                {
                    const uint8_t contact_id = message.report_id - t100_subsequent_report_ids[0];
                    int event = (message.data[0] & 0xf);
                    uint16_t x = message.data[1] | (message.data[2] << 8);
                    uint16_t y = message.data[3] | (message.data[4] << 8);
                    bool tip = false
                    if (event == DOWN)
                    {
                        digitizer_report.fingers[contact_id].tip = true;
                    }
                    if (event == UP || event == UNSUP || event == DOWNUP)
                    {
                        digitizer_report.fingers[contact_id].tip = 0;
                    }
                    digitizer_report.fingers[contact_id].confidence = !(event == SUP || event == DOWNSUP);
                    if (event != UP)
                    {
                        digitizer_report.fingers[contact_id].x = x;
                        digitizer_report.fingers[contact_id].y = y;
                    }
                }
                else
                {
                    printf("Unhandled ID: %d (%d..%d) %d\n", message.report_id, t100_subsequent_report_ids[0], t100_subsequent_report_ids[t100_num_reports], t100_subsequent_report_ids[t100_num_reports - 1]);
                }
            }
        }
    }
    return digitizer_report;
}
