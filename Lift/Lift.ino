const unsigned int pinSwUp =  2; /*핀 번호*/
const unsigned int pinSwDn =  3; /**/

const unsigned int pinDirA =  4; /* 시계방향 회전 */
const unsigned int pinDirB =  5; /* 반시계방향 회전 */
const unsigned int pinPWM  =  6;

int pwmValue  =  255; /* 회전값은 0 ~ 255 까지 있다 */

void setup() {
  pinMode(pinSwDn, INPUT_PULLUP); /* 아두니오 보드에 있는 명령어 */ 
  pinMode(pinSwUp, INPUT_PULLUP);
  pinMode(pinDirA, OUTPUT      );
  pinMode(pinDirB, OUTPUT      );
  
  while(!Serial);
  Serial.begin(115200);
}

void _up() {  /* 리프트가 올라갈떄 회전 방향 */
  digitalWrite(pinDirA, LOW );
  digitalWrite(pinDirB, HIGH);
  analogWrite(pinPWM, pwmValue);
}

void _down() { /* 리프트가 내려갈떄 화전의 방향 */
  digitalWrite(pinDirA, HIGH);
  digitalWrite(pinDirB, LOW );
  analogWrite(pinPWM, pwmValue);
}

void _stop() {
  digitalWrite(pinDirA, HIGH);
  digitalWrite(pinDirB, HIGH);
  analogWrite(pinPWM, LOW);
}

void loop() { /* 여기가 main 문 ==> 1 누르면 올라가고 0을 누르면 내려감 */
  if(Serial.available() > 0) {
    
    unsigned int ch = Serial.read();
  
    if     (ch == '1'){
      Serial.println("up");  _up();
      while(digitalRead(pinSwUp) == HIGH){
        if(Serial.read() == 'q')  break;
      }
      _stop();
    }
    
    else if(ch == '0'){
      Serial.println("down"); _down();
      while(digitalRead(pinSwDn) == HIGH) {
        if(Serial.read() == 'q')  break;
      }
      _stop(); }
      
    else if(ch == 'q'){   /* 중간에 비상 정지 버튼 */
      Serial.println("stop"); _stop(); }
  }
}

