/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONS
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
// Include custom libraries
#include "vga_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1.h"

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// character array
char screentext[40];

// draw speed
int threshold = 10 ;

int motor_disp = 0; 

// #define multfix15(a,b) ((fix15)((((signed long long)(a))*((signed long long)(b)))>>15))
// #define float2fix15(a) ((fix15)((a)*32768.0)) // 2^15
// #define fix2float15(a) ((float)(a)/32768.0)
// #define absfix15(a) abs(a) 
// #define int2fix15(a) ((fix15)(a << 15))
// #define fix2int15(a) ((int)(a >> 15))
// #define char2fix15(a) (fix15)(((fix15)(a)) << 15)
// #define divfix(a,b) (fix15)(div_s64s64( (((signed long long)(a)) << 15), ((signed long long)(b))))
// #define max(a, b) (a >= b ? a : b)
// #define min(a, b) (a < b ? a : b)

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)

// fix15 oneeightyoverpi = float2fix15(57.2957795);
// fix15 zeropt001 = float2fix15(.001);
// fix15 zeropt999 = float2fix15(.999);


// semaphore
static struct pt_sem vga_semaphore ;

// Some paramters for PWM
#define WRAPVAL 5000 //PWM Wrap value for PWM counter
#define CLKDIV  25.0 //PWM clock division - control ticks
uint slice_num ; //slice value for GPIO 4 and 5 - PWM functionality

volatile fix15 complementary_angle; //anglular displacement of the wheel system
volatile fix15 desired_angle = int2fix15(6); //initial calibratin and angular displacement
volatile fix15 error_accum = 0; //accumulation variable
volatile fix15 kP; //P term
volatile fix15 kI; //I term
volatile fix15 kD; //D term
volatile fix15 I_max = int2fix15(1500); //cap value for error accumulator
volatile fix15 angle_increment = float2fix15(0.001); //dithering value not used in this case
volatile fix15 prev_error = 0; //to track error from previous iterations
/*
The individual contributions of each term in the controller
*/
volatile fix15 p_contrib;
volatile fix15 d_contrib;
volatile fix15 i_contrib;

// PWM duty cycle
volatile int control1 ; //positive side of the motor PWM
volatile int control2; //negative side of the motor PWM

/*
To implement the double bugger variable. 
*/
volatile int old_control1 ; 
volatile int old_control2 ;


// Interrupt service routine
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.

    // Update duty cycle
    if (control1!=old_control1 || control2 != old_control2 ) {
        old_control1 = control1 ;
        old_control2 = control2 ;

        pwm_set_chan_level(slice_num, PWM_CHAN_B, control2);
        pwm_set_chan_level(slice_num, PWM_CHAN_A, control1);
    }

    // Gather measurements
    mpu6050_read_raw(acceleration, gyro);


    // Accelerometer angle (degrees - 15.16 fixed point)
    fix15 accel_angle = multfix15(divfix(acceleration[0], acceleration[1]), oneeightyoverpi) ;

    // Gyro angle delta (measurement times timestep) (15.16 fixed point)
    fix15 gyro_angle_delta = multfix15(gyro[2], zeropt001) ;

    // Complementary angle (degrees - 15.16 fixed point)
    complementary_angle = multfix15(complementary_angle - gyro_angle_delta, zeropt999) + multfix15(accel_angle, zeropt001) ;

    fix15 error = desired_angle - complementary_angle;
    error_accum += error;
    //error accumulator that limits the variable's value to avoid integrator windup. 
    if (error_accum > I_max){
        error_accum = I_max;
    }else if (error_accum < -I_max){
        error_accum = -I_max;
    }
    //To minimize the integrator windup when passing through the middle region. 
    if (error_accum < abs(1)){
        error_accum = 0;
    }

    //Value to multiply D with.
    fix15 error_deriv = error - prev_error;
    /*
    Individual contributions of each term in the PID controller. 
    */
    p_contrib = multfix15(error, kP); 
    i_contrib = multfix15(error_accum, kI);
    d_contrib = multfix15(error_deriv, kD);
    int duty_cycle = fix2int15(p_contrib + i_contrib + d_contrib);
    
    //Limits the duty_cycle o stay withiin the range of [-WRAPVAL WRAPVAL] and hold one variable to 0 while the other one has the PWM VALUE
    if (duty_cycle > WRAPVAL){
        control1 = WRAPVAL;
        control2 = 0;
    }else if (duty_cycle < -1 * WRAPVAL){
        control1 = 0;
        control2 = WRAPVAL;
    }else if (duty_cycle >= 0){
        control1 = abs(duty_cycle);
        control2 = 0;
    }else{
        control1 = 0;
        control2 = abs(duty_cycle);
    }
    prev_error = error;
    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

// Thread that draws to VGA display
static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // We will start drawing at column 81
    static int xcoord = 81 ;
    
    // Rescale the measurements for display
    static float OldRange = 500. ; // (+/- 250)
    static float NewRange = 150. ; // (looks nice on VGA)
    static float OldMin = -250. ;
    static float OldMax = 250. ;

    static char comp_angle[20];
    static char motor_control[20];
    static char Kp_contrib[20] = "Kp contrib: ";
    static char Ki_contrib[20] = "Ki contrib: ";
    static char Kd_contrib[20] = "Kd contrib: ";


    static char val1[20];
    static char val2[20];
    static char val3[20];


    // Control rate of drawing
    static int throttle ;

    // Draw the static aspects of the display
    setTextSize(1) ;
    setTextColor(WHITE);

    // Draw bottom plot
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "+5000") ;
    setCursor(50, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "-5000") ;
    setCursor(50, 425) ;
    writeString(screentext) ;

    // Draw top plot
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "0") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "+250") ;
    setCursor(45, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "-250") ;
    setCursor(45, 225) ;
    writeString(screentext) ;



    while (true) {
        // Wait on semaphore
        /*
        Displaying the complementary angle
        */
        fillRect(0, 0, 400, 50, BLACK);
        PT_SEM_WAIT(pt, &vga_semaphore);
        setCursor(10, 10) ;
        sprintf(comp_angle, "%f", fix2float15(complementary_angle) ); 
        writeString(comp_angle) ;

        /*
        Displaying the motor control
        */
        setCursor(10, 30) ;
        motor_disp = motor_disp + (((control2 - control1) - motor_disp)>>6) ;
        sprintf(motor_control, "%d", motor_disp ); 
        writeString(motor_control) ;
        

        /*
        Displaying individual contributions of the PID controller
        */
        setCursor(120, 10) ;
        writeString(Kp_contrib) ;
        sprintf(val1, "%f", fix2float15(p_contrib) ); 
        writeString(val1) ;
        
        setCursor(120, 20) ;
        writeString(Ki_contrib) ;
        sprintf(val2, "%f", fix2float15(i_contrib) ); 
        writeString(val2) ;

        setCursor(120, 30) ;
        writeString(Kd_contrib) ;
        sprintf(val3, "%f", fix2float15(d_contrib) ); 
        writeString(val3) ;


        // Increment drawspeed controller
        throttle += 1 ;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) { 
            // Zero drawspeed controller
            throttle = 0 ;

            // Erase a column
            drawVLine(xcoord, 0, 480, BLACK) ;

            // Draw bottom plot (multiply by 120 to scale from +/-5000 - Motor Plot
            drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[0])*120.0)-OldMin)/OldRange)), WHITE) ;
            drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[1])*120.0)-OldMin)/OldRange)), RED) ;
            drawPixel(xcoord, 430 - (int)(NewRange*((float)((fix2float15(acceleration[2])*120.0)-OldMin)/OldRange)), GREEN) ;
            
            drawPixel(xcoord, 430 - (int)(NewRange*((float)(((motor_disp)*.048)-OldMin)/OldRange)), YELLOW) ;


            // Draw top plot
            // drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[0]))-OldMin)/OldRange)), WHITE) ;
            // drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[1]))-OldMin)/OldRange)), RED) ;
            // drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[2]))-OldMin)/OldRange)), GREEN) ;
            //Draw the top plot +/-250 - Complementary angle plot
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(complementary_angle)*5)-OldMin)/OldRange)), BLUE) ;


/*
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[0]))-OldMin)/OldRange)), WHITE) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[1]))-OldMin)/OldRange)), RED) ;
            drawPixel(xcoord, 230 - (int)(NewRange*((float)((fix2float15(gyro[2]))-OldMin)/OldRange)), GREEN) ;
*/
            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
            }
        }
    }
    // Indicate end of thread
    PT_END(pt);
}

// User input thread. User can change draw speed
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char classifier = 'c';
    static int test_in1 ;
    static int test_in2 ;
    static int test_in3 ; 
    static int test_in4 ;

    static float float_in ;
    while(1) {
        sprintf(pt_serial_out_buffer, "input a command: ");
        serial_write ;
        // spawn a thread to do the non-blocking serial read
        serial_read ;
        // convert input string to number
        /*
        Reading the values - desired angle, P, I, D
        */
        sscanf(pt_serial_in_buffer,"%d %d %d %d", &test_in4, &test_in1, &test_in2, &test_in3) ;
        kP = int2fix15(test_in1); 
        kI = int2fix15(test_in2); 
        kD = int2fix15(test_in3); 
        desired_angle = int2fix15(test_in4); 


        // MANUAL SET CONTROL 
        // if (test_in1 > 5000 || test_in2 > 5000 ) continue ;
        // else if (test_in1 < 0 || test_in2 < 0) continue ;
        // else { 
        //     control1 = test_in1;
        //     control2 = test_in2;
        // }
        // num_independents = test_in ;
        if (classifier=='t') {
            sprintf(pt_serial_out_buffer, "timestep: ");
            serial_write ;
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%d", &test_in1) ;
            if (test_in1 > 0) {
                threshold = test_in1 ;
            }
        }
    }
    PT_END(pt) ;
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;
}

int main() {

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;
    gpio_pull_up(SDA_PIN) ;
    gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial) ;
    pt_schedule_start ;

}
