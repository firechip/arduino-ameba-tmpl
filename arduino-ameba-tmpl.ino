#include <lvgl.h>
// #include "lv_examples.h"
#include "SPI.h"
#include "AmebaILI9341.h"
#include <Adafruit_FT6206.h>

#define TFT_RESET  5
#define TFT_DC     9
#define TFT_CS    10

AmebaILI9341 tft = AmebaILI9341(TFT_CS, TFT_DC, TFT_RESET);
#define ILI9341_SPI_FREQUENCY 50000000

Adafruit_FT6206 ts = Adafruit_FT6206();

static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.setAddress(area->x1, area->y1, area->x2, area->y2);
  //tft.pushColors(&color_p->full, w * h, true);
  uint16_t *pImg = &color_p->full;
  uint32_t mcu_pixels = w * h;
  digitalWrite(TFT_CS, LOW);
  // push all the image block pixels to the screen
  digitalWrite(TFT_DC, HIGH);
  while (mcu_pixels--) {
    uint16_t pixel = *pImg++;
    SPI.transfer(pixel >> 8);
    SPI.transfer(pixel & 0xFF);
  }
  digitalWrite(TFT_CS, HIGH);

  lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
bool my_touchpad_read(lv_indev_drv_t * indev_driver, lv_indev_data_t * data)
{
  bool touched = ts.touched();

  if (!touched) {
    data->state = LV_INDEV_STATE_REL;
  } else {
    data->state = LV_INDEV_STATE_PR;

    TS_Point p = ts.getPoint();
    p.x = map(p.x, 0, 240, 240, 0);
    p.y = map(p.y, 0, 320, 320, 0);
    int y = tft.getHeight() - p.x;
    int x = p.y;
    /*Set the coordinates*/
    data->point.x = x;
    data->point.y = y;

    Serial.print("Data x");
    Serial.println(x);
    Serial.print("Data y");
    Serial.println(y);
  }

  return false; /*Return `false` because we are not buffering and no more data to read*/
}

void setup()
{
  Serial.begin(115200); /* prepare for possible serial debug */
  SPI.setDefaultFrequency(ILI9341_SPI_FREQUENCY);

  lv_init();
 

  tft.begin(); /* TFT init */
  tft.setRotation(1); /* Landscape orientation */
  if (!ts.begin(40)) {
    Serial.println("Unable to start touchscreen.");
  }
  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

  /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 320;
  disp_drv.ver_res = 240;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the (dummy) input device driver*/
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  //Try an example from the lv_examples repository
  //https://github.com/lvgl/lv_examples
  // lv_demo_widgets();
  lv_ex_label_1();
}

void loop()
{

  lv_task_handler(); /* let the GUI do its work */
  delay(5);
  __asm__("BKPT");
}

void lv_ex_label_1(void)
{
    lv_obj_t * label1 = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_long_mode(label1, LV_LABEL_LONG_BREAK);     /*Break the long lines*/
    lv_label_set_recolor(label1, true);                      /*Enable re-coloring by commands in the text*/
    lv_label_set_align(label1, LV_LABEL_ALIGN_CENTER);       /*Center aligned lines*/
    lv_label_set_text(label1, "#0000ff Re-color# #ff00ff words# #ff0000 of a# label "
                              "and  wrap long text automatically.");
    lv_obj_set_width(label1, 150);
    lv_obj_align(label1, NULL, LV_ALIGN_CENTER, 0, -30);

    lv_obj_t * label2 = lv_label_create(lv_scr_act(), NULL);
    lv_label_set_long_mode(label2, LV_LABEL_LONG_SROLL_CIRC);     /*Circular scroll*/
    lv_obj_set_width(label2, 150);
    lv_label_set_text(label2, "It is a circularly scrolling text. ");
    lv_obj_align(label2, NULL, LV_ALIGN_CENTER, 0, 30);
}