//Sabanci yarismasi icin kod

//Digital pins
int auto_start_button = 4;
int wb_sensor;
int led_pin = 5;

bool is_auto;
bool color;
float distance;

void setup() {
    //Pins
    pinMode(auto_start_button, INPUT);
    pinMode(wb_sensor, INPUT);
    pinMode(led_pin, OUTPUT);
    
    //Set everything to zero
    is_auto = wb_sensor = color = distance = 0;

  
    
    Serial.begin(9600);
}

void pid(d){
    
  
}

void loop() {
    //Autonomous
    if(is_auto){
        //If it sees a different tape from previous one
        if(color != digitalRead(wb_sensor)){
            distance += 5.0; //add length of tape
            color = !color; //reverse the color
        }

        if(distance >= 1500.0){
            pid(distance);  
        }

        //Autonomus led - green
        
    }else{
        if(digitalRead(auto_start_button)){
            is_auto = 1;  
        }  

        //Autonomous led - red
    }
    delay(20);
}
