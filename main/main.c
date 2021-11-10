#include "esp_log.h"

#include "my_app.h"
#include "mqtt.h"
#include "esp_http_server.h"

#include "driver/pwm.h"


static const char *TAG = "USER_APP";



//curl "http://iot:10000/set?br=0.55&ct=3500&sp=100"
//curl --data-binary "@./build/light.bin" -X POST "http://192.168.61.251/fw"

//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
#include <math.h>

typedef struct _fvalue_t{
  float value;
  float min_value;
  float max_value;
}fvalue_t;
int fvalue_init( fvalue_t* fvalue, float val, float minval, float maxval);
int fvalue_set( fvalue_t* fvalue, float val);
float fvalue_get( fvalue_t* fvalue);
float fvalue_get_range( fvalue_t* fvalue);


int fvalue_init( fvalue_t* fvalue, float val, float minval, float maxval) {
  fvalue->min_value = minval;
  fvalue->max_value = maxval;
  return fvalue_set( fvalue, val);
}
int fvalue_initX01( fvalue_t* fvalue, float val) {
  fvalue->min_value = 0.0;
  fvalue->max_value = 1.0;
  return fvalue_set( fvalue, val);
}
int fvalue_init001( fvalue_t* fvalue) {
  fvalue->value = 0.0;
  fvalue->min_value = 0.0;
  fvalue->max_value = 1.0;
  return 0;
}
int fvalue_init101( fvalue_t* fvalue) {
  fvalue->value = 1.0;
  fvalue->min_value = 0.0;
  fvalue->max_value = 1.0;
  return 0;
}
int fvalue_set( fvalue_t* fvalue, float val) {
  fvalue->value = fmax( fmin( val, fvalue->max_value), fvalue->min_value);
  return 0;
}
float fvalue_get( fvalue_t* fvalue) {
  return fvalue->value;
}
float fvalue_get_range( fvalue_t* fvalue) {
  return ( fvalue->max_value - fvalue->min_value);
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
typedef enum{
  CMD_NONE,
  CMD_SET,
  CMD_FADEIN,
  CMD_FADEOUT
}COMMANDS;

typedef struct _LIGHT_RT{
  SemaphoreHandle_t sem;
  //------------------------------------------------
  COMMANDS cmd;   /* command: NONE/SET/FADEIN/FADEOUT */
  fvalue_t cmd_br;  /* bright: 0.0 - 1.0 */
  fvalue_t cmd_ct;  /* color temperature: 3000K - 5000K */
  fvalue_t cmd_sp;  /* t for br[0-100%] */
  //------------------------------------------------
  fvalue_t cur_br;  /* bright: 0.0 - 1.0 */
  fvalue_t cur_ct;  /* color temperature: 3000K - 5000K */
}LIGHT_RT;
static LIGHT_RT _light_rt;

int init_rt(){
  if( _light_rt.sem != NULL) {
    return 0;
  }
  _light_rt.sem = xSemaphoreCreateBinary(); xSemaphoreGive( _light_rt.sem);

  _light_rt.cmd = CMD_NONE;
  fvalue_initX01( &_light_rt.cmd_br, 0);
  fvalue_init( &_light_rt.cmd_ct, 0, 3000, 5000);
  fvalue_init( &_light_rt.cmd_sp, 0, 0.1, 2*3600);
  fvalue_initX01( &_light_rt.cur_br, 0);
  fvalue_init( &_light_rt.cur_ct, 0, 3000, 5000);
  if( _light_rt.sem != NULL) {
    return 0;
  }
  return -1;
}
LIGHT_RT* lock_rt(){
  if( _light_rt.sem != NULL) {
    if( pdTRUE == xSemaphoreTake( _light_rt.sem, portMAX_DELAY)) {
      return &_light_rt;
    }
  }
  return NULL;
}
int free_rt( LIGHT_RT* rt){
  if( rt != NULL) {
    if( rt->sem != NULL) {
      xSemaphoreGive( rt->sem);
      return 0;
    }
  }
  return -1;
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------


#define PWM_0_OUT_IO_NUM    5
#define PWM_1_OUT_IO_NUM    4
#define SW1_IN_IO_NUM       0 /*2*/
#define PWM_PERIOD    (3000)  /* PWM period 3000us(333Hz), same as depth */
// pwm pin number
const uint32_t pin_num[2] = {
    PWM_0_OUT_IO_NUM,
    PWM_1_OUT_IO_NUM
};

// duties table, real_duty = duties[x]/PERIOD
uint32_t duties[2] = {
    0, 0
};

// phase table, delay = (phase[x]/360)*PERIOD
float phase[2] = {
    0, 0
};

static void light_task(){
  init_rt();

  pwm_init( PWM_PERIOD, duties, 2, pin_num);
  pwm_set_phases( phase);
  pwm_start();

  while( 1){
    LIGHT_RT* rt = lock_rt();
    if( rt != NULL){
      // Input section
      float cmd_br = fvalue_get( &rt->cmd_br);
      float cmd_ct = fvalue_get( &rt->cmd_ct);
      float cmd_sp = fvalue_get( &rt->cmd_sp);
      float cur_br = fvalue_get( &rt->cur_br);
      float cur_ct = fvalue_get( &rt->cur_ct);
      float br_range = fvalue_get_range( &rt->cur_br);
      float ct_range = fvalue_get_range( &rt->cur_ct);

      // Calk section
      switch( rt->cmd){
        case CMD_SET:
          cur_br = cmd_br;
          cur_ct = cmd_ct;
          rt->cmd = CMD_NONE;
          break;

        case CMD_FADEIN:
          if( cur_br >= cmd_br) { // Disable FADE routine
            cmd_br = cur_br;
          }
          break;

        case CMD_FADEOUT:
          if( cur_br <= cmd_br) { // Disable FADE routine
            cmd_br = cur_br;
          }
          break;

        case CMD_NONE:
        default:
          break;
      }
      // FADE_IN_OUT routine
      if( cur_br < cmd_br) {
        cur_br = fmin( ( cur_br+br_range/cmd_sp), cmd_br);
      }else if( cur_br > cmd_br ){
        cur_br = fmax( ( cur_br-br_range/cmd_sp), cmd_br);
      }

      if( cur_ct < cmd_ct) {
        cur_ct = fmin( ( cur_ct+ct_range/cmd_sp), cmd_ct);
      }else if( cur_ct > cmd_ct ){
        cur_ct = fmax( ( cur_ct-ct_range/cmd_sp), cmd_ct);
      }

      // Output section
      fvalue_set( &rt->cur_br, cur_br);
      fvalue_set( &rt->cur_ct, cur_ct);

      cur_br = fvalue_get( &rt->cur_br);
      cur_ct = fvalue_get( &rt->cur_ct);

      cur_br = powf( cur_br, 1.5);

      float ka = ( 5000 - cur_ct) / 1000;
      float kb = ( cur_ct - 3000) / 1000;
      ka = fmin( fmax( ka, 0.0), 1.0);
      kb = fmin( fmax( kb, 0.0), 1.0);
      float pwm_a = cur_br * ka;
      float pwm_b = cur_br * kb;

//printf( "B: %f %f\r\n", pwm_a, pwm_b);

      // Set PWMs channels
      vTaskSuspendAll();
      pwm_set_duty( 0, pwm_a*PWM_PERIOD);
      pwm_set_duty( 1, pwm_b*PWM_PERIOD);
      pwm_start();
      xTaskResumeAll();

      free_rt( rt);
    }
    vTaskDelay( 1000 / portTICK_RATE_MS);
  }
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

#define PARAM_CMD     "cmd"
#define PARAM_BRIGHT  "br"
#define PARAM_CTEMP   "ct"
#define PARAM_SPEED   "sp"
esp_err_t common_handler(httpd_req_t *req)
{
  char*  buf;
  size_t buf_len;
  /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = malloc(buf_len);

    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      ESP_LOGI(TAG, "URL query => %s", buf);

      char value[32];
      LIGHT_RT* rt = lock_rt();
      if( rt != NULL){
        if (httpd_query_key_value(buf, PARAM_BRIGHT, value, sizeof(value)) == ESP_OK) {
            ESP_LOGI(TAG, "Parameter br=%s", value);
            rt->cmd  = ( COMMANDS)req->user_ctx;
            fvalue_set( &rt->cmd_br, atof( value)); // pv->T = strtod( &ack_data[ 3], &next_tag);
        }
        if (httpd_query_key_value(buf, PARAM_CTEMP, value, sizeof(value)) == ESP_OK) {
            ESP_LOGI(TAG, "Parameter ct=%s", value);
            rt->cmd  = ( COMMANDS)req->user_ctx;
            fvalue_set( &rt->cmd_ct, atof( value));
        }
        if (httpd_query_key_value(buf, PARAM_SPEED, value, sizeof(value)) == ESP_OK) {
            ESP_LOGI(TAG, "Parameter sp=%s", value);
            rt->cmd  = ( COMMANDS)req->user_ctx;
            fvalue_set( &rt->cmd_sp, atof( value));
        }
        free_rt( rt);
      }
    }
    free(buf);
  }

  httpd_resp_set_status( req, HTTPD_200); //#define HTTPD_200      "200 OK"                     /*!< HTTP Response 200 */
  httpd_resp_send( req, "OK", 2);
  //    httpd_resp_set_status( req, HTTPD_204); //#define HTTPD_204      "204 No Content"             /*!< HTTP Response 204 */
  //    httpd_resp_send( req, NULL, 0);
  //    httpd_resp_set_status( req, HTTPD_400); //#define HTTPD_400      "400 Bad Request"            /*!< HTTP Response 400 */
  //    httpd_resp_send( req, "BlaBlaBla", -1);
  return ESP_OK;
}

httpd_uri_t set_req = {
    .uri       = "/set",
    .method    = HTTP_GET,
    .handler   = common_handler,
    .user_ctx  = ( void*)CMD_SET
};
httpd_uri_t fadein_req = {
    .uri       = "/fadein",
    .method    = HTTP_GET,
    .handler   = common_handler,
    .user_ctx  = ( void*)CMD_FADEIN
};
httpd_uri_t fadeout_req = {
    .uri       = "/fadeout",
    .method    = HTTP_GET,
    .handler   = common_handler,
    .user_ctx  = ( void*)CMD_FADEOUT
};

esp_err_t httpd_register_user_uri_handlers( httpd_handle_t server)
{
  esp_err_t ret;
  ESP_LOGI(TAG, "LIGHT HTTPD Enabled:");
  ESP_LOGI(TAG, "    curl \"http://[DEVICE IP]/set?br=1.0&ct=5000&sp=0\"");
  ESP_LOGI(TAG, "    curl \"http://[DEVICE IP]/fadein?br=1.0&ct=5000&sp=3600\"");
  ESP_LOGI(TAG, "    curl \"http://[DEVICE IP]/fadeout?br=0.3&ct=3000&sp=3600\"");
  ret = httpd_register_uri_handler( server, &set_req);
  if( ret != ESP_OK)
    return ret;
  ret = httpd_register_uri_handler( server, &fadein_req);
  if( ret != ESP_OK)
    return ret;
  ret = httpd_register_uri_handler( server, &fadeout_req);
  return ret;
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

esp_err_t mqtt_event_user_handler_cb( esp_mqtt_event_handle_t event)
{
  return ESP_OK;
}
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------

esp_err_t user_app_main()
{
  ESP_LOGI( TAG, "LIGHT");
  xTaskCreate( light_task, "light_task", 2048, NULL, 10, NULL);
  return ESP_OK;
}
