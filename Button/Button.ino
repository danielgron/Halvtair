/*
  Button

 Turns on and off a light emitting diode(LED) connected to digital
 pin 13, when pressing a pushbutton attached to pin 2.


 The circuit:
 * LED attached from pin 13 to ground
 * pushbutton attached to pin 2 from +5V
 * 10K resistor attached to pin 2 from ground

 * Note: on most Arduinos there is already an LED on the board
 attached to pin 13.


 created 2005
 by DojoDave <http://www.0j0.org>
 modified 30 Aug 2011
 by Tom Igoe

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/Button
 */

// constants won't change. They're used here to
// set pin numbers:
const int io1 = 9;     // the number of the pushbutton pin
const int io2 = 8;
const int io3 = 7;
const int io4 = 6;
const int io5 = 5;
const int io6 = 4;
const int io7 = 3;
const int io8 = 2;

const int dec = 10;
const int inc = 11;
const int tickButton = 12;

const int mem1 = 14;
const int mem2 = 15;
const int mem3 = 16;
const int mem4 = 17;
const int mem5 = 18;
const int mem6 = 19;

const int spIp = A6;
const int edit = A7;

const int ledPin =  13;      // the number of the LED pin


int ipViewed = 0;
int spViewed = 0;

int switchSpIp;
int switchEdit;
int tickBtn;
int incBtn;
int decBtn;

byte instr;
int ip=0;
int sp=0;
byte regA=0;
byte regB=0;
byte data[64] = {};
bool flag;
bool halt;
bool hasStarted; 


// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

void setup() {
  ip=0;
  sp=0;
  switchSpIp = analogRead(spIp);
  switchEdit = analogRead(edit);
  incBtn = digitalRead(inc);
  decBtn = digitalRead(dec);
  tickBtn = digitalRead(tickButton);
  if (switchSpIp  && switchEdit) loadFact();
  if (switchSpIp && !switchEdit) loadFactNonTail();
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  setRedIO();

  pinMode(mem1, OUTPUT);
  pinMode(mem2, OUTPUT);
  pinMode(mem3, OUTPUT);
  pinMode(mem4, OUTPUT);
  pinMode(mem5, OUTPUT);
  pinMode(mem6, OUTPUT);


  
  
  Serial.begin(9600);
  digitalWrite(ledPin, HIGH);
  delay(1000);
  digitalWrite(ledPin, LOW);
  delay(1000);
  testLights();
  
}

void loop() {
  setRedIO();
  switchSpIp = analogRead(spIp);
  switchEdit = analogRead(edit);
  incBtn = digitalRead(inc);
  decBtn = digitalRead(dec);
  tickBtn = digitalRead(tickButton);
  if (tickBtn && !hasStarted) {
    hasStarted = true;
    ip =0;
  }

  
  // read the state of the pushbutton value:
  //buttonState = digitalRead(2)*digitalRead(3)*digitalRead(4)*digitalRead(5)*digitalRead(6)*digitalRead(7)*digitalRead(8)*digitalRead(9);
  //Serial.println(digitalRead(2)+digitalRead(3)*2+digitalRead(4)*4+digitalRead(5)*8+digitalRead(6)*16+digitalRead(7)*32+digitalRead(8)*64+digitalRead(9)*128);

//Testing the buttons. If Led13 lights on arduino when button is on, the system registers it correctly  
//buttonState = digitalRead(io1)||digitalRead(io2)|| digitalRead(io3)|| digitalRead(io4)|| digitalRead(io5)|| digitalRead(io6)|| digitalRead(io7)|| digitalRead(io8)|| digitalRead(tickButton)|| digitalRead(dec)|| digitalRead(inc)|| analogRead(spIp)|| analogRead(edit);
  // check if the pushbutton is pressed.

  
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }
  
  memoryLights();
  if ( tickBtn && !halt) {
   tick();
   delay(400);
  }
  if (!hasStarted &&  switchEdit && !halt && incBtn) {
    saveInstruction();
  }
  if ((halt | !hasStarted) && (incBtn || decBtn)) incDec();
  delay(100); 
}
void loadFact(){
  data[0] = B01000010;
  data[1] = B00010000;
  data[2] = B01001010;
  data[3] = B00010000;
  data[4] = B00001100;
  data[5] = B11001000;
  data[6] = B00010010;
  data[7] = B00001111;
  data[8] = B00110010;
  data[9] = B00000111;
  data[10] = B10001100;
  data[11] = B00011001;
  data[12] = B00110101;
  data[13] = B00000010;
  data[14] = B00100010;
  data[15] = B00110010;
  data[16] = B00010111;
  data[17] = B00100001;
  data[18] = B00001100;
  data[19] = B10001000;
}

void loadFactNonTail(){ // Få skrevet koden 
data[0] = B01001010;
data[1] = B00010000;
data[2] = B00001100;
data[3] = B11000110;
data[4] = B00010010;
data[5] = B00001111;
data[6] = B00110010;
data[7] = B00000111;
data[8] = B10001100;
data[9] = B01000010;
data[10] = B00100001;
data[11] = B00011000;
data[12] = B00010000;
data[13] = B00010111;
data[14] = B00010000;
data[15] = B00001100;
data[16] = B11000110;
data[17] = B00010011;
data[18] = B00010010;
data[19] = B00000010;
data[20] = B00100001;
data[21] = B00011000;
}

void saveInstruction(){
  byte instr = B00000000;
  if (digitalRead(io1)) instr = (instr | B00000001);
  if (digitalRead(io2)) instr = (instr | B00000010);
  if (digitalRead(io3)) instr = (instr | B00000100);
  if (digitalRead(io4)) instr = (instr | B00001000);
  if (digitalRead(io5)) instr = (instr | B00010000);
  if (digitalRead(io6)) instr = (instr | B00100000);
  if (digitalRead(io7)) instr = (instr | B01000000);
  if (digitalRead(io8)) instr = (instr | B10000000);
  data[ip] = instr;
}

void incDec(){
if (switchSpIp){
if (incBtn)incSp();
else if (decBtn) decSp();
}
else{
if (incBtn) incIp();
else if (decBtn) decIp();
}
delay(300);
}

void setAnalogInput(){
  
}
void testLights(){
  digitalWrite(mem1, HIGH);
  digitalWrite(mem2, HIGH);
  digitalWrite(mem3, HIGH);
  digitalWrite(mem4, HIGH);
  digitalWrite(mem5, HIGH);
  digitalWrite(mem6, HIGH);
  delay(1000);
  digitalWrite(mem1, LOW);
  digitalWrite(mem2, LOW);
  digitalWrite(mem3, LOW);
  digitalWrite(mem4, LOW);
  digitalWrite(mem5, LOW);
  digitalWrite(mem6, LOW);
}

void memoryLights(){
  byte mem;
  byte bSP;
  if (switchSpIp){
    mem = data[sp];
    bSP = sp;
  }
  else {
  mem = data[ip];
    bSP = ip;
  }
    
    //Serial.println("Instruction in memory ");
    //Serial.println(instr);
Serial.print("sp in memory ");
Serial.println(sp);
Serial.print("sp value ");
Serial.println(data[sp]);
Serial.print("flag ");
Serial.println(flag);

// Write out SP
    if ((bSP & B00000001) == B00000001) digitalWrite(mem1, HIGH);
    else digitalWrite(mem1, LOW);
    if ((bSP & B00000010) == B00000010) digitalWrite(mem2, HIGH);
    else digitalWrite(mem2, LOW);
    if ((bSP & B00000100) == B00000100) digitalWrite(mem3, HIGH);
    else digitalWrite(mem3, LOW);
    if ((bSP & B00001000) == B00001000) digitalWrite(mem4, HIGH);
    else digitalWrite(mem4, LOW);
    if ((bSP & B00010000) == B00010000) digitalWrite(mem5, HIGH);
    else digitalWrite(mem5, LOW);
    if ((bSP & B00100000) == B00100000) digitalWrite(mem6, HIGH);
    else digitalWrite(mem6, LOW);

    // write out data
    if ((mem & B00000001) == B00000001) digitalWrite(io1, HIGH);
    else digitalWrite(io1, LOW);
    if ((mem & B00000010) == B00000010) digitalWrite(io2, HIGH);
    else digitalWrite(io2, LOW);
    if ((mem & B00000100) == B00000100) digitalWrite(io3, HIGH);
    else digitalWrite(io3, LOW);
    if ((mem & B00001000) == B00001000) digitalWrite(io4, HIGH);
    else digitalWrite(io4, LOW);
    if ((mem & B00010000) == B00010000) digitalWrite(io5, HIGH);
    else digitalWrite(io5, LOW);
    if ((mem & B00100000) == B00100000) digitalWrite(io6, HIGH);
    else digitalWrite(io6, LOW);
    if ((mem & B01000000) == B01000000) digitalWrite(io7, HIGH);
    else digitalWrite(io7, LOW);
    if ((mem & B10000000) == B10000000) digitalWrite(io8, HIGH);
    else digitalWrite(io8, LOW);
  if (halt) delay(100);
  
}

void setRedIO(){
  if (switchEdit){
  pinMode(io1, INPUT);
  pinMode(io2, INPUT);
  pinMode(io3, INPUT);
  pinMode(io4, INPUT);
  pinMode(io5, INPUT);
  pinMode(io6, INPUT);
  pinMode(io7, INPUT);
  pinMode(io8, INPUT);
  }
  else {
  pinMode(io1, OUTPUT);
  pinMode(io2, OUTPUT);
  pinMode(io3, OUTPUT);
  pinMode(io4, OUTPUT);
  pinMode(io5, OUTPUT);
  pinMode(io6, OUTPUT);
  pinMode(io7, OUTPUT);
  pinMode(io8, OUTPUT);
  }
}

void tick(){
instr = data[ip];
//digitalRead(2)+digitalRead(3)*2+digitalRead(4)*4+digitalRead(5)*8+digitalRead(6)*16+digitalRead(7)*32+digitalRead(8)*64+digitalRead(9)*128; // get instr
Serial.print("Instruction in tick for ip ");
Serial.println(ip);
delay(50);
Serial.println(instr);
//0000 0000  NOP IP++
if (instr==B00000000) incIp();
//0000 0001  ADD A ← A + B; IP++
else if (instr==B00000001) {regA=regA+regB; incIp();}
//0000 0010  MUL A ← A*B; IP++
else if (instr==B00000010) {regA=regA*regB; incIp();}
//0000 0011  DIV A ← A/B; IP++
else if (instr==B00000011) {regA/regB; incIp();}
//0000 0100  ZERO  F ← A = 0; IP++
else if (instr==B00000100) {flag = regA==0; incIp();}
//0000 0101  NEG F ← A < 0; IP++
else if (instr==B00000101) {flag = regA<0; incIp();}
//0000 0110  POS F ← A > 0; IP++
if (instr==B00000110) {flag = regA>0; incIp();}
//0000 0111  NZERO F ← A ≠ 0; IP++
if (instr==B00000111) {flag = regA!=0; incIp();}
//0000 1000  EQ  F ← A = B; IP++
if (instr==B00001000) {flag = regA==regB; incIp();}
//0000 1001  LT  F ← A < B; IP++
if (instr==B00001001) {flag = regA < regB; incIp();}
// 0000 1010  GT  F ← A > B; IP++
if (instr==B00001010) {flag =regA > regB; incIp();}
//0000 1011  NEQ F ← A ≠ B; IP++
if (instr==B00001011) {flag = regA !=regB; incIp();}
//0000 1100  ALWAYS  F ← true; IP++
if (instr==B00001100) {flag =true; incIp();}
//0000 1101    Undefined
if (instr==B00000000) {}
//0000 1110    Undefined
if (instr==B00000000) {}
//0000 1111  HALT  Halts execution
if (instr==B00001111) {halt = true;Serial.println(data[63]);}
//0001 000r  PUSH r  [--SP] ← r; IP++
if ((instr & B11111110)==B00010000) {
  decSp(); 
  if ((instr & B00001000) == B00000000) data[sp]=regA;
  else data[sp]=regB;
  incIp();
  }
//0001 001r  POP r r ← [SP++]; IP++
if ((instr & B11111110)==B00010010) {
  Serial.println("pop 0001 001r ");
  if ((instr & B00000001) == B00000000) regA =data[sp];
  else regB = data[sp];
  incSp();
  incIp();
  }
//0001 0100  MOV A B B ← A; IP++
if (instr==B00010100) {regB=regA; incIp();Serial.println("MOV A B");}
//0001 0101  MOV B A A ← B; IP++
if (instr==B00010101) {regA=regB; incIp();Serial.println("MOV B A ");}

//0001 0110  INC A++; IP++
if (instr == B00010110) {regA++; incIp();}
//0001 0111  DEC A--; IP++
if (instr == B00010111) {regA--; incIp();}
// 0001 1ooo  RTN +o  IP ← [SP++]; SP += o; IP++
if ((instr & B11111000)==B00011000 ) {
  Serial.println("RTN");
  ip = data[sp];
  incSp();
  sp = sp + (instr & B00000111);
  incIp();
  }
// 0010 r ooo  MOV r o    [SP + o] ← r; IP++
if ((instr & B11110000) ==B00100000) {
  Serial.println("Mov r o");
  if ((instr & B00001000) == B00000000) data[sp + (instr & B00000111)] = regA ;
  else data[sp+ (instr & B00000111)] = regB;
  incIp();
  }
// 0011 ooor  MOV o r r ← [SP + o]; IP++
if ((instr & B11110000)==B00110000) {
  Serial.println("Mov o r");
  byte o = (instr & B00001110) >>1;
  if ((instr & B00000001)== B00000000) regA=data[sp+o];
  else regB=data[sp+o];
  incIp();
  }
//01vv vvvr  MOV v r r ← v; IP++
if (( instr & B11000000) == B01000000) {
  Serial.println("Mov v r");
  if ((instr & B00000001) == B00000000) regA = ((instr & B00111110)>>1);
  else regB = ((instr & B00111110)>>1);
  incIp();
  }
// 10aa aaaa  JMP #a  if F then IP ← a else IP++
if (( instr & B11000000) == B10000000) {
  Serial.println("JMP to");
  if (flag){
    
    ip = (instr & B00111111);
    Serial.println(ip);
  }
  else incIp();
  }
//11aa aaaa  CALL #a if F then [--SP] ← IP; IP ← a else IP++
if ((instr & B11000000) == B11000000) {
  Serial.println("Call");
  if (flag){
    decSp();
    data[sp] = ip;
    ip = (instr & B00111111);
  }
  else incIp();
  }
}
void incIp(){
  if (ip==63) ip = 0;
  else ip++;
}
void decIp(){
  if (ip==0) ip = 63;
  else ip--;
}
void incSp(){
  if (sp==63) sp = 0;
  else sp++;
}
void decSp(){
  if (sp==00) sp = 63;
  else sp--;
}

