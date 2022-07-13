/*
 * @Author: lebhoryi@gmail.com
 * @Date: 2021-06-30 16:52:58
 * @LastEditors: lebhoryi@gmail.com
 * @LastEditTime: 2021-07-01 19:36:20
 * @Version: V0.0.1
 * @FilePath: /art_pi/applications/fire_detection.c
 * @Description: fire detection demo app
 */
#include <rtthread.h>
#include "drv_common.h"
#include <rtdevice.h>
#include "drv_spi_ili9488.h"  // spi lcd driver
#include <lcd_spi_port.h>  // lcd ports

/* fire detection */
#include <rt_ai_fruit_model.h>
#include <rt_ai.h>
#include <rt_ai_log.h>
#include <backend_cubeai.h>
#include <logo.h>

#define LED_PIN GET_PIN(I, 8)
#define PWM_PIN GET_PIN(B,1)
void servo_open(void);//机械爪打开
void servo_close(void);//机械爪关闭
char *fruit_label[] = {"Apple Braeburn","Apple Granny Smith","Apricot","Avocado","Banana","Blueberry","Cactus fruit","Cantaloupe","Cherry","Clementine","Corn","Cucumber Ripe","Grape Blue","Kiwi","Lemon","Limes","Mango","Onion White","Orange","Papaya","Passion Fruit","Peach","Pear","Pepper Green","Pepper Red","Plum","pineapple","Pomegranate","Potato Red","Raspberry","Strawberry","Tomato","Watermelon"};
struct rt_event ov2640_event;
rt_ai_buffer_t ai_flag = 0;

void ai_run_complete(void *arg){
    *(int*)arg = 1;
}

void ai_camera();
void bilinera_interpolation(rt_uint8_t *in_array, short height, short width,
                            rt_uint8_t *out_array, short out_height, short out_width);

int fruit_classification(void){
    /* init spi data notify event */
    rt_event_init(&ov2640_event, "ov2640", RT_IPC_FLAG_FIFO);

    struct drv_lcd_device *lcd;
    struct rt_device_rect_info rect_info = {0, 0, LCD_WIDTH, 240};
    lcd = (struct drv_lcd_device *)rt_device_find("lcd");

    lcd_show_image(0, 0, 320, 240, LOGO);
    lcd_show_string(90, 140, 16, "Hello RT-Thread!");
    lcd_show_string(90, 156, 16, "Fruit classification");
    rt_thread_mdelay(1000);

//    rt_err_t result = RT_EOK;
    int ai_run_complete_flag = 0;

    /* find a registered model handle */
    static rt_ai_t model = NULL;
    model = rt_ai_find(RT_AI_FRUIT_MODEL_NAME);
    if(!model) {rt_kprintf("ai model find err\r\n"); return -1;}

    // allocate input memory
    rt_ai_buffer_t *input_image = rt_malloc(RT_AI_FRUIT_IN_1_SIZE_BYTES);
    if (!input_image) {rt_kprintf("malloc err\n"); return -1;}

    // allocate calculate memory
    rt_ai_buffer_t *work_buf = rt_malloc(RT_AI_FRUIT_WORK_BUFFER_BYTES);
    if (!work_buf) {rt_kprintf("malloc err\n"); return -1;}

    // allocate output memory
    rt_ai_buffer_t *_out = rt_malloc(RT_AI_FRUIT_OUT_1_SIZE_BYTES);
    if (!_out) {rt_kprintf("malloc err\n"); return -1;}

    // ai model init
    rt_ai_buffer_t model_init = rt_ai_init(model, work_buf);
    if (model_init != 0) {rt_kprintf("ai init err\n"); return -1;}
    rt_ai_config(model, CFG_INPUT_0_ADDR, input_image);
    rt_ai_config(model, CFG_OUTPUT_0_ADDR, _out);

    ai_camera();

    while(1)
    {
        rt_pin_write(LED_PIN, PIN_LOW);
        rt_event_recv(&ov2640_event,
                            1,
                            RT_EVENT_FLAG_AND | RT_EVENT_FLAG_CLEAR,
                            RT_WAITING_FOREVER,
                            RT_NULL);
        rt_pin_write(LED_PIN, PIN_HIGH);
        lcd->parent.control(&lcd->parent, RTGRAPHIC_CTRL_RECT_UPDATE, &rect_info);
        if (ai_flag > 0)
        {
            // resize
            bilinera_interpolation(lcd->lcd_info.framebuffer, 240, 320, input_image, 32, 32);
            rt_ai_run(model, NULL, NULL);
            uint8_t *out = (uint8_t *)rt_ai_output(model, 0);
            int max_index = 0;
                    for(int i = 1 ; i < 33 ; i++){
                         if(out[i] > out[max_index]){
                             max_index = i;
                         }
                    }
            lcd_show_string(10, 172, 16, "Fruit prediction:");
            lcd_show_string(210, 172, 16, fruit_label[max_index]);
            if(max_index==4)//识别到香蕉
            {
                servo_open();//机械爪打开
                rt_thread_mdelay(1000);
                servo_close();
            }
        }
        DCMI_Start();

    }
    rt_free(input_image);
    rt_free(work_buf);
    rt_free(_out);

    return 0;
}
MSH_CMD_EXPORT(fruit_classification,fire demo);
//INIT_COMPONENT_EXPORT(fire_app);


void ai_camera()
{
    rt_gc0328c_init();
    ai_flag = 1;
    DCMI_Start();
}


int is_in_array(short x, short y, short height, short width)
{
    if (x >= 0 && x < width && y >= 0 && y < height)
        return 1;
    else
        return 0;
}


void bilinera_interpolation(rt_uint8_t* in_array, short height, short width,
                            rt_uint8_t* out_array, short out_height, short out_width)
{
    double h_times = (double)out_height / (double)height,
           w_times = (double)out_width / (double)width;
    short  x1, y1, x2, y2, f11, f12, f21, f22;
    double x, y;

    for (int i = 0; i < out_height; i++){
        for (int j = 0; j < out_width*3; j=j+3){
            for (int k =0; k <3; k++){
                x = j / w_times + k;
                y = i / h_times;

                x1 = (short)(x - 3);
                x2 = (short)(x + 3);
                y1 = (short)(y + 1);
                y2 = (short)(y - 1);
                f11 = is_in_array(x1, y1, height, width*3) ? in_array[y1*width*3+x1] : 0;
                f12 = is_in_array(x1, y2, height, width*3) ? in_array[y2*width*3+x1] : 0;
                f21 = is_in_array(x2, y1, height, width*3) ? in_array[y1*width*3+x2] : 0;
                f22 = is_in_array(x2, y2, height, width*3) ? in_array[y2*width*3+x2] : 0;
                out_array[i*out_width*3+j+k] = (rt_uint8_t)(((f11 * (x2 - x) * (y2 - y)) +
                                           (f21 * (x - x1) * (y2 - y)) +
                                           (f12 * (x2 - x) * (y - y1)) +
                                           (f22 * (x - x1) * (y - y1))) / ((x2 - x1) * (y2 - y1)));
            }
        }
    }
}
void servo_open(void){
    for(int i=0;i<10;i++){
        rt_thread_mdelay(18.1);
        rt_pin_write(PWM_PIN, PIN_HIGH);
        rt_thread_mdelay(1.9);
        rt_pin_write(PWM_PIN, PIN_LOW);
    }
}//控制舵机打开

void servo_close(void){
    for(int i=0;i<10;i++){
        rt_thread_mdelay(18);
        rt_pin_write(PWM_PIN, PIN_HIGH);
        rt_thread_mdelay(2);
        rt_pin_write(PWM_PIN, PIN_LOW);
    }
}//控制舵机关闭
