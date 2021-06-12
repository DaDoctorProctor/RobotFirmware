#include "dl_lib_matrix3d.h"
#include <esp32-hal-ledc.h>
int speed = 255;
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "Arduino.h"
//Extra//
#include "camera_index.h"
extern int val_final;
extern String RGBCommand;


typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
    return 0;
  }
  j->len += len;
  return len;
}

static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  int64_t fr_start = esp_timer_get_time();

  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");

  size_t out_len, out_width, out_height;
  uint8_t * out_buf;
  bool s;
  {
    size_t fb_len = 0;
    if (fb->format == PIXFORMAT_JPEG) {
      fb_len = fb->len;
      res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    } else {
      jpg_chunking_t jchunk = {req, 0};
      res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
      httpd_resp_send_chunk(req, NULL, 0);
      fb_len = jchunk.len;
    }
    esp_camera_fb_return(fb);
    int64_t fr_end = esp_timer_get_time();
    Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start) / 1000));
    return res;
  }

  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  if (!image_matrix) {
    esp_camera_fb_return(fb);
    Serial.println("dl_matrix3du_alloc failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  out_buf = image_matrix->item;
  out_len = fb->width * fb->height * 3;
  out_width = fb->width;
  out_height = fb->height;

  s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
  esp_camera_fb_return(fb);
  if (!s) {
    dl_matrix3du_free(image_matrix);
    Serial.println("to rgb888 failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  jpg_chunking_t jchunk = {req, 0};
  s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
  dl_matrix3du_free(image_matrix);
  if (!s) {
    Serial.println("JPEG compression failed");
    return ESP_FAIL;
  }

  int64_t fr_end = esp_timer_get_time();
  return res;
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];
  dl_matrix3du_t *image_matrix = NULL;

  static int64_t last_frame = 0;
  if (!last_frame) {
    last_frame = esp_timer_get_time();
  }

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      {
        if (fb->format != PIXFORMAT_JPEG) {
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if (!jpeg_converted) {
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) {
      break;
    }
    int64_t fr_end = esp_timer_get_time();
    int64_t frame_time = fr_end - last_frame;
    last_frame = fr_end;
    frame_time /= 1000;
    /*Serial.printf("MJPG: %uB %ums (%.1ffps)\n",
                  (uint32_t)(_jpg_buf_len),
                  (uint32_t)frame_time, 1000.0 / (uint32_t)frame_time
                 );*/
  }

  last_frame = 0;
  return res;
}


static esp_err_t cmd_handler(httpd_req_t *req)
{
  char*  buf;
  size_t buf_len;
  char variable[32] = {0,};
  char value[32] = {0,};

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if (!buf) {
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
          httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  int val = atoi(value);
  sensor_t * s = esp_camera_sensor_get();
  int res = 0;

  if (!strcmp(variable, "framesize"))
  {
    Serial.println("framesize");
    if (s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
  }
  else if (!strcmp(variable, "quality"))
  {
    Serial.println("quality");
    res = s->set_quality(s, val);
  }


//--------------------------------------------------------------
  //Remote Control Car
  //Don't use channel 1 and channel 2
  else if (!strcmp(variable, "flash"))
  {ledcWrite(7, val);}
  else if (!strcmp(variable, "sm1")){
    //ledcWrite(3,val);
    val_final = 1000 + val;
  }
  else if (!strcmp(variable, "sm2")){
    //ledcWrite(4, val);
    val_final = 2000 + val;
  }
  else if (!strcmp(variable, "sm3")){
    //ledcWrite(5, val);
    val_final = 3000 + val;
  }
  else if (!strcmp(variable, "sm4")){
    //ledcWrite(6, val);
    //int val_04 = 4000 + val;
    val_final = 4000 + val;
  }
  else if (!strcmp(variable, "smR")){
    RGBCommand = "RE" + String(val);
  }
  else if (!strcmp(variable, "smG")){
    RGBCommand = "GE" + String(val);
  }
  else if (!strcmp(variable, "smB")){
    RGBCommand = "BU" + String(val);
  }
  else if (!strcmp(variable, "load")){
    val_final = 1000 + val;
  }
  else if (!strcmp(variable, "home")){
    
  }
  
  else if (!strcmp(variable, "car")) {
    if (val == 1) {
      //Serial.println("Forward");
      ledcWrite(8, 60);
      Serial.println("arriba");
    }
    else if (val == 2) {
      //Serial.println("Turn Left");
      ledcWrite(8, 120);
      Serial.println("izquierdo");
    }
    else if (val == 3) {
      //Serial.println("Stop");
      ledcWrite(8, 0);
      Serial.println("stop");
    }
    else if (val == 4) {
      //Serial.println("Turn Right");
      ledcWrite(8, 200);
      Serial.println("derecho");
    }
    else if (val == 5) {
      //Serial.println("Backward");
      ledcWrite(8, 255);
      Serial.println("abajo");
    }
  }
  else
  {
    Serial.println("variable");
    res = -1;
  }

  if (res) {
    return httpd_resp_send_500(req);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}


static esp_err_t status_handler(httpd_req_t *req) {
  static char json_response[1024];

  sensor_t * s = esp_camera_sensor_get();
  char * p = json_response;
  *p++ = '{';

  p += sprintf(p, "\"framesize\":%u,", s->status.framesize);
  p += sprintf(p, "\"quality\":%u,", s->status.quality);
  *p++ = '}';
  *p++ = 0;
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, strlen(json_response));
}

/* Index page */
static const char PROGMEM INDEX_HTML[] = R"rawliteral(

<!doctype html>
<html>
    <head>
		<meta charset="utf-8">
		<meta name="viewport" content="width=device-width,initial-scale=1">
        <title>Robot modular - GDF-X1905A</title>
        <style>
            .button {background-color: #000000;
				border: none;
				border-radius: 4px;
				color: white;
				padding: 2vh 3vh;
				text-align: center;
				font-size: 3.5vh;
				margin: 4px 2px;
				cursor: pointer;
				width: 20%;
				border: 2px solid black;
				}
			
			/* Slider properties*/
			
			.container {
			  width: 20vw;
			  height: 20vh;
			  display: flex;
			  align-items: center;
			  justify-content: center;
			  
			}

			.range {
			  display: flex;
			  width: 90vw;
			}

			.range__slider {
			  width: 60vw;
			}

			.range__value {
			  width: 45%;
			  margin-left: 10vw;    
			  text-align: center;
			  border-left: #e6e4e4 1px solid;
			}

			.form-group {
			  display: flex;
			  flex-direction: column; 
			  justify-content: center;
			}

			.form-group label {
			  text-transform: uppercase;
			  font-size: .7rem;
			  color: #222;
			  margin-bottom: 5px;
			}

			.form-group span {
			  font-size: 2rem;
			  font-weight: 600;
			  color: #3c3b3b;
			}

			.range__slider label {
			  margin-bottom: 10px;
			}

			.range__slider [type="range"] {
			  width: 50vw;
			  -webkit-appearance: none;
			  height: 13px;
			  border-radius: 6px;
			  background: #f1f1f1;
			  outline: none;
			  padding: 0;
			  margin: 0;
			
			}

			/* custom thumb */
			.range__slider [type="range"]::-webkit-slider-thumb {
			  -webkit-appearance: none;
			  appearance: none;
			  width: 8vw;
			  height: 8vw;
			  border-radius: 50%;
			  background: #7a00ff;
			  border: #7a00ff 5px solid;
			  cursor: pointer;
			  -webkit-transition: background .15s ease-in-out;
			  transition: background .15s ease-in-out;
			}

			.range__slider [type="range"]::-webkit-slider-thumb:hover {
			  background: #7a00ff;
			}

			.range__slider [type="range"]::-moz-range-thumb {
			  width: 60px;
			  height: 20px;
			  border: 0;
			  border-radius: 50%;
			  background: #f0932b;
			  border: #f9ca24 5px solid; cursor: pointer;
			  -webkit-transition: background .15s ease-in-out;
			  transition: background .15s ease-in-out;
			}

			.range__slider [type="range"]::-moz-range-thumb:hover {
			  background: #f9ca24;
			}

			/* remove border */
			input::-moz-focus-inner, input::-moz-focus-outer {
			  border: 0;
			}
			
			
			.label {color: #000000;
				font-size: 3vh;
				}
				
			.writing_txt {color: #000000;
				font-size: 2vh;
				}

			body { font-family: Arial, sans-serif;
				margin: 0; padding: 0;
				background: black;
				overflow: hidden;
				};
      
			.button1 {
				background-color: black; 
				color: white;
				}

			.button1:active {
				background-color: white;
				color: black;
				border: 2px solid black;
				}
				
			.buttonS2{
				background-color: black; 
				color: white;
				width: 30vw;
				height: 10vh;
				
				}

			.buttonS2:active {
				background-color: white;
				color: black;
				border: 2px solid black;
				}
      
			/* Style the tab */
			.tab {
				overflow: hidden;
				background-color: black;
				border: 0px
				}

			/* Style the buttons inside the tab */
			.tab button {
				background-color: black;
				color: white;
				float: left;
				border: none;
				outline: none;
				cursor: pointer;
				padding: 0px 3.3vh;
				transition: 0.3s;
				font-size: 1.5vh;
				height: 3.6vh;
				}

			/* Change background color of buttons on hover */
			.tab button:hover {
				background-color: #ddd;
				}

			/* Create an active/current tablink class */
			.tab button.active {
				background-color: #ccc;
				}

			/* Style the tab content */
			.tabcontent {
				display: none;
				background-color: white;
				height: 40vh;
				}
      
			/* RGB BAR */
			.wrapper{
				height: 1.5vh;
				width: 100%;
				position: relative;
				background: linear-gradient(135deg, #14ffe9, #ffeb3b, #ff00e0);
				border-radius: 0px;
				cursor: default;
				animation: animate 1.5s linear infinite;
				}

			.wrapper .display,
			.wrapper span{
				position: absolute;
				top: 50%;
				left: 50%;
				transform: translate(-50%, -50%);
				}
				
			.wrapper .display{
				z-index: 999;
				height: 30px;
				width: 20px;
				background: #1b1b1b;
				border-radius: 6px;
				text-align: center;
				}
	
			.display #doc{
				line-height: 2vh;
				color: #fff;
				font-size: 2vh;
				font-weight: 1;
				letter-spacing: 1px;
				background: linear-gradient(135deg, #14ffe9, #ffeb3b, #ff00e0);
				-webkit-background-clip: text;
				-webkit-text-fill-color: transparent;
				animation: animate 1.5s linear infinite;
				}
			.display #julian{
				line-height: 2vh;
				color: #fff;
				font-size: 2vh;
				font-weight: 1;
				letter-spacing: 1px;
				background: linear-gradient(135deg, #14ffe9, #ffeb3b, #ff00e0);
				-webkit-background-clip: text;
				-webkit-text-fill-color: transparent;
				animation: animate 1.5s linear infinite;
				}
			.display #grunt{
				line-height: 2vh;
				color: #fff;
				font-size: 2vh;
				font-weight: 1;
				letter-spacing: 1px;
				background: linear-gradient(135deg, #14ffe9, #ffeb3b, #ff00e0);
				-webkit-background-clip: text;
				-webkit-text-fill-color: transparent;
				animation: animate 1.5s linear infinite;
				}
			.display #ventura{
				line-height: 2vh;
				color: #fff;
				font-size: 2vh;
				font-weight: 1;
				letter-spacing: 1px;
				background: linear-gradient(135deg, #14ffe9, #ffeb3b, #ff00e0);
				-webkit-background-clip: text;
				-webkit-text-fill-color: transparent;
				animation: animate 1.5s linear infinite;
				}
			
			.display #peter{
				line-height: 2vh;
				color: #fff;
				font-size: 2vh;
				font-weight: 1;
				letter-spacing: 1px;
				background: linear-gradient(135deg, #14ffe9, #ffeb3b, #ff00e0);
				-webkit-background-clip: text;
				-webkit-text-fill-color: transparent;
				animation: animate 1.5s linear infinite;
				}
				
			@keyframes animate {
				100%{
				filter: hue-rotate(360deg);}
				}
				
			.wrapper span{
				height: 10%;
				width: 100%;
				border-radius: 10px;
				background: inherit;
				}
			.wrapper span:first-child{
				filter: blur(7px);
				}
			.wrapper span:last-child{
				filter: blur(20px);
				}
	
			img {
				display: block;
				}
	
			/*Align the border of the sliders*/
			.sliderAlign {
				padding-left: 4vh;
				padding-top: 1%;
				}
				
			.SpecsAlign {
				float: left;
				width: 75vh;
				padding-left: 10vh;
				padding-top: 1%;
				}
			
			.DataAlign {
				float: left;
				width: 75vh;
				padding-left: 15vh;
				padding-top: 3%;
				}
    
			/* Style the close button */
			.closetab {
				float: right;
				cursor: pointer;
				font-size: 20px;
				height: 20px;
				color: black
				}

			.closetab:hover {color: red;}
			
			.sliderValueBox {
				background-color: lightgrey;
				display:inline-block;
				width: 9vh;
				height: 4vh;
				border-radius: 2vh;
				
			}
			
			.sliderValueBoxHex {
				background-color: lightgrey;
				display:inline-block;
				width: 14vh;
				height: 4vh;
				border-radius: 2vh;
				
			}
			
			.sliderTextBox {
				background-color: white;
				display:inline-block;
				width: 5vh;
				height: 4vh;
				border-radius: 0vh;
				
			}
			
			.sliderWhiteSpace {
				background-color: white;
				display:inline-block;
				width: 12vh;
				height: 4vh;
				border-radius: 0vh;
				
			}
			
			#sliderColorBox{
				background-color: black;
				display:inline-block;
				width: 20vh;
				height: 8vh;
			}
			
			.backgroundGrey{
				background-color: gray;
			}
			
			.noselect {
				-webkit-touch-callout: none; /* iOS Safari */
				-webkit-user-select: none; /* Safari */
				-khtml-user-select: none; /* Konqueror HTML */
				-moz-user-select: none; /* Old versions of Firefox */
				-ms-user-select: none; /* Internet Explorer/Edge */
				user-select: none; /* Non-prefixed version, currently
				supported by Chrome, Opera and Firefox */
			}
		
		</style>
    </head>
    <body>
	
		<div align=center> 
			<img src='http://192.168.4.1:81/stream' style='width:100%; transform:rotate(180deg);'>
		</div>
    
		<div class="wrapper"></div> 
		
		<div class="tab">
			<button class="tablinks" onclick="tabAdder(event, 'Control');myDIV.style.display='none'" id="defaultOpen">Controles</button>
			<button class="tablinks" onclick="tabAdder(event, 'Servomotors');myDIV.style.display='none'">Servomotores</button>
			<button class="tablinks" onclick="tabAdder(event, 'Illumination');myDIV.style.display='none'">Iluminacion</button>
			<button class="tablinks" onclick="tabAdder(event, 'About');myDIV.style.display='none'">Acerca de</button>
			
			<span onclick="myDIV.style.display='block'"; class="closetab" id="defaultOpen">&times</span>
			
			<button class="tablinks" onclick="tabAdder(event, 'Debug')" id="myDIV">Debug</button>
			
			
			
		</div>

		<div id="Control" class="tabcontent">
			<div class = noselect>
				<br/>
				<div align=center> 
					<button class="button button1" id="forward" ontouchstart="fetch(document.location.origin+'/control?var=car&val=1');" 
					ontouchend="fetch(document.location.origin+'/control?var=car&val=3');" >△</button>
				</div>
		  
				<div align=center>  
					<button class="button button1" id="turnleft" ontouchstart="fetch(document.location.origin+'/control?var=car&val=2');" 
					ontouchend="fetch(document.location.origin+'/control?var=car&val=3');" >◁</button>
					<button class="button button1" id="turnright" ontouchstart="fetch(document.location.origin+'/control?var=car&val=4');" 
					ontouchend="fetch(document.location.origin+'/control?var=car&val=3');" >▷</button>     
				</div>
			
				
				<div align=center> 
					<button class="button button1" id="backward" ontouchstart="fetch(document.location.origin+'/control?var=car&val=5');" 
					ontouchend="fetch(document.location.origin+'/control?var=car&val=3');">▽</button>
				</div>
			</div>
		</div>
		<div id="Servomotors" class="tabcontent">
			<br/>
			
			<div class="sliderAlign"> 
				<form class="range">
					<div class="sliderWhiteSpace">
						<div class="sliderValueBox">
							<div align=center> 
								<label class="label" id="convSM1">25</label>
							</div>
						</div>
					</div>
					
					<div class="form-group range__slider">
						<input id="sm1" type="range" class="slider" min="0" max="255" value = "64"
						onchange="try{fetch(document.location.origin+'/control?var=sm1&val='+this.value);}catch(e){};" 
						oninput="Conversion(sm1.value,1);setBlue(sm1.value);colorBox()">
					</div>
		
					<div class="sliderTextBox">
						<div align=center> 
							<label class="label">H1</label>
						</div>
					</div>
				</form>
			</div>
			<br/>
		
			<div class="sliderAlign"> 
				<form class="range">
					<div class="sliderWhiteSpace">
						<div class="sliderValueBox">
							<div align=center> 
								<label class="label" id="convSM2">25</label>
							</div>
						</div>
					</div>
					
					<div class="form-group range__slider">
						<input type="range" class="slider" id="sm2" min="0" max="255" value="64" 
						onchange="try{fetch(document.location.origin+'/control?var=sm2&val='+this.value);}catch(e){}" 
						oninput="Conversion(sm2.value,2);setBlue(sm2.value);colorBox()">
					</div>
					
					<div class="sliderTextBox">
						<div align=center> 
							<label class="label">H2</label>
						</div>
					</div>
					
				</form>
			</div>
			
			<br/>
		
			<div class="sliderAlign"> 
				<form class="range">
				
					<div class="sliderWhiteSpace">
						<div class="sliderValueBox">
							<div align=center> 
								<label class="label" id="convSM3">25</label>
							</div>
						</div>
					</div>
					
					<div class="form-group range__slider">
						<input type="range" class="slider" id="sm3" min="0" max="255" value="64" 
						onchange="try{fetch(document.location.origin+'/control?var=sm3&val='+this.value);}catch(e){}" 
						oninput="Conversion(sm3.value,3);setBlue(sm3.value);colorBox()">
					</div>
					
					<div class="sliderTextBox">
						<div align=center> 
							<label class="label">H3</label>
						</div>
					</div>
				</form>
			</div>
			
			<br/>
			
			<button class="button buttonS2" id="setHome" ontouchstart="setHome()" 
				ontouchend="setHome();sm1.value = getCuPosH1();sm2.value = getCuPosH2();sm3.value = getCuPosH3();" >Home</button>
			<button class="button buttonS2" id="savePosition" ontouchstart="" 
				ontouchend="savePos(sm1.value,sm2.value,sm3.value)" >Save</button>
			<button class="button buttonS2" id="loadPosition" ontouchstart="" 
				ontouchend="sm1.value = loadPosH1();sm2.value = loadPosH2();sm3.value = loadPosH3();" >Load</button>
					
		</div>
	
		<div id="Illumination" class="tabcontent">
			<br/>
			
				<div class="sliderAlign"> 
					<form class="range">
					
						<div class="sliderWhiteSpace">
							<div class="sliderValueBox">
								<div align=center> 
									<label class="label" id="convSMR">0</label>
								</div>
							</div>
						</div>
						
						<div class="form-group range__slider">
							<input type="range" class="slider" id="smR" min="0" max="255" value="0" 
							onchange="try{fetch(document.location.origin+'/control?var=smR&val='+this.value);}catch(e){}" 
							oninput="Conversion(smR.value,4);setRed(smR.value);colorBox()">
						</div>
						
						<div class="sliderTextBox">
							<div align=center> 
								<label class="label">R</label>
							</div>
						</div>
						
					</form>
				</div>
			<br/>
		
			<div class="sliderAlign"> 
				<form class="range">
				
					<div class="sliderWhiteSpace">
						<div class="sliderValueBox">
							<div align=center> 
								<label class="label" id="convSMG">0</label>
							</div>
						</div>
					</div>
					
					
					<div class="form-group range__slider">
						<input type="range" class="slider" id="smG" min="0" max="255" value="0" 
						onchange="try{fetch(document.location.origin+'/control?var=smG&val='+this.value);}catch(e){}" 
						oninput="Conversion(smG.value,5);setGreen(smG.value);colorBox()">
					</div>
					
					<div class="sliderTextBox">
						<div align=center> 
							<label class="label">G</label>
						</div>
					</div>
				</form>
			</div>
			<br/>
		
			<div class="sliderAlign"> 
				<form class="range">
					<div class="sliderWhiteSpace">
						<div class="sliderValueBox">
							<div align=center> 
								<label class="label" id="convSMB">0</label>
							</div>
						</div>
					</div>
					
					<div class="form-group range__slider">
						<input type="range" class="slider" id="smB" min="0" max="255" value="0" 
						onchange="try{fetch(document.location.origin+'/control?var=smB&val='+this.value);}catch(e){}" 
						oninput="Conversion(smB.value,6);setBlue(smB.value);colorBox()">
					</div>
							
					<div class="sliderTextBox">
						<div align=center> 
							<label class="label">B</label>
						</div>
					</div>
				</form>
			</div>
			<br/>
			
			<div align=center>
				<div id = "sliderColorBox"></div>
				<div class="sliderValueBoxHex">
					<label class="label" id="convHex">#000000</label>
				</div>
			</div>
				
			
		</div>
		
		<div id="About" class="tabcontent">
			<div class="SpecsAlign"> 

				
				<div class="display">
					<div id="julian"></div>
				</div>
				<div class="display">
					<div id="grunt"></div>
				</div>
				<div class="display">
					<div id="ventura"></div>
				</div>
				<div class="display">
					<div id="peter"></div>
				</div>
				<div class="display">
					<div id="doc"></div>
				</div>
				
				
				<label class="label writing_txt">Version: GDF-X1905A</label>
				
			</div>	
			
		</div>
  
  
		<div id="Debug" class="tabcontent">
			
			<div align=center> 
				<button class="button button1" id="foward" 
				onclick="fetch(document.location.origin+'/control?var=car&val=1');">
			△</button>
			</div>
		  
			<div align=center>  
				<button class="button button1" id="turnleft" 
				onclick="fetch(document.location.origin+'/control?var=car&val=2');">
				◁</button>
				<button class="button button1" id="stop" 
				onclick="fetch(document.location.origin+'/control?var=car&val=3');">
				X</button>
				<button class="button button1" id="turnright" 
				onclick="fetch(document.location.origin+'/control?var=car&val=4');">
				▷</button>
			  
			</div>
			
			<div align=center> 
				<button class="button button1" id="backward"
				onclick="fetch(document.location.origin+'/control?var=car&val=5');">
				▽</button>
			</div>
			
		</div>
		
		<script>
			var xtz = 0;
			var last_tabName = "";
			
			function tabAdder(evt, tabName) {
				var i, tabcontentZERO, tablinksZERO;
				
				tabcontentZERO = document.getElementsByClassName("tabcontent");
				
				var tabcontentArray = Array.from(tabcontentZERO);
				
				var tabcontent = tabcontentArray;
				
				//Get the tabs orderned in different parts
				for (i = 0; i < tabcontent.length; i++) {
					tabcontent[i].style.display = "none";
				}
				
				tablinksZERO = document.getElementsByClassName("tablinks");
				
				var tablinksArray = Array.from(tablinksZERO);
				var tablinks = tablinksArray;
				
				//Check for active tab color, name bar, do not modify.
				for (i = 0; i < tablinks.length; i++) {
					tablinks[i].className = tablinks[i].className.replace(" active", "");
				}
				
				//document.getElementById(tabName).style.display = "block";
				
				document.getElementById(tabName).style.display = "block";
				//Controls the Background latch.
				evt.currentTarget.className += " active";
				
			}
			
			document.getElementById("defaultOpen").click();
					
			function myFunction() {
				var x = document.getElementById("myDIV");
				if (x.style.display === "block") {
					x.style.display = "none";
				} else {
					x.style.display = "block";
				}
			}
			
			function Conversion(val,name){
				var x = val;
				var z = parseInt(x * 100 / 255) ;
				switch (name) {
					case 0:
						document.getElementById("convFlash").innerHTML = z;
						break;
					case 1:
						document.getElementById("convSM1").innerHTML = z;
						break;
					case 2:
						document.getElementById("convSM2").innerHTML = z;
						break;
					case 3:
						document.getElementById("convSM3").innerHTML = z;
						break;
					case 4:
						document.getElementById("convSMR").innerHTML = x;
						break;
					case 5:
						document.getElementById("convSMG").innerHTML = x;	
						break;
					case 6:
						document.getElementById("convSMB").innerHTML = x;			
						break;
					}
			}			
			
			//Slider home
			var H1;
			var H2;
			var H3;
			function setHome(){
				H1 = 64;
				H2 = 64;
				H3 = 64; 
				document.getElementById("convSM1").innerHTML = 25;
				document.getElementById("convSM2").innerHTML = 25;
				document.getElementById("convSM3").innerHTML = 25;
			};		
			function getCuPosH1(){return H1};
			function getCuPosH2(){return H2};
			function getCuPosH3(){return H3};
			
			//Save and load 
			var saveH1 = 64;
			var saveH2 = 64;
			var saveH3 = 64;
			function savePos(a,b,c){
				saveH1 = a;
				saveH2 = b;
				saveH3 = c;
			}
			function loadPosH1(){Conversion(saveH1,1); return saveH1}
			function loadPosH2(){Conversion(saveH2,2); return saveH2}
			function loadPosH3(){Conversion(saveH3,3); return saveH3}
			
			//RGB Color names
			setInterval(()=>{
				const doc = document.querySelector(".display #doc");
				const julian = document.querySelector(".display #julian");
				doc.textContent = "Homero Perez Mata";
				julian.textContent = "Ramon Julian Garza Rios";
				grunt.textContent = "Ivan Maldonado Amador";
				ventura.textContent = "Abigail Rodriguez Ventura";
				peter.textContent = "Josue Ricardo Guzman Santos";
			}); 
			
			//RGB Color Picker
			
			var SoldierRed = 0;
			var SoldierGreen = 0;
			var SoldierBlue = 0;
			
			function getRed(){
				var red =SoldierRed;
				return red
			}
			
			function setRed(r){
				SoldierRed = r;		
			}
			
			function getGreen(){
				var red =SoldierGreen;
				return red
			}
			
			function setGreen(r){
				SoldierGreen = r;		
			}
			
			function getBlue(){
				var red = SoldierBlue;
				return red
			}
			
			function setBlue(r){
				SoldierBlue = r;		
			}
			
			function colorBox(){
				var SoldierHex = rgb2Hex(getRed(),getGreen(),getBlue());
				document.getElementById("convHex").innerHTML = SoldierHex;
				
				var x = document.getElementById("sliderColorBox");
				x.style.backgroundColor = SoldierHex;
			}
			
			//Reserve functions
			function rgb2Hex(red, green, blue) {
			  const rgb = (red << 16) | (green << 8) | (blue << 0);
			  return '#' + (0x1000000 + rgb).toString(16).slice(1);
			}
		
		</script>
    
    </body>
</html>

)rawliteral";

static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

//Main Loop
void startCameraServer()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_uri_t index_uri = {
        .uri       = "/",
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t status_uri = {
        .uri       = "/status",
        .method    = HTTP_GET,
        .handler   = status_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t cmd_uri = {
        .uri       = "/control",
        .method    = HTTP_GET,
        .handler   = cmd_handler,
        .user_ctx  = NULL
    };

   httpd_uri_t stream_uri = {
        .uri       = "/stream",
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };
    
    Serial.printf("Starting web server on port: '%d'\n", config.server_port);
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
    }

    config.server_port += 1;
    config.ctrl_port += 1;
    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}
