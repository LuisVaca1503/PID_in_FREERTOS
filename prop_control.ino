#include <definitions.h>
#include <math.h>

// Parametros del lazo de  control

const float h = 0.01;  // tiempo de muestreo en segundos
const float umin = -5; // valor mínimo de la señal de control
const float umax = 5; // valor máximo de la señal de control
const float deadZone = 0.3; // valor estimado de la zona muerta 

// variables del lazo de control
float reference; 
//Calculo kp
float SP=25;
float tr=0.5;
float z;
float wn;
float kp;
float ki;
float kd; 
float y; // angulo a controlar
float u; //accion de control
float e; // error
float usat; // señal de control saturada

//Ancho de banda del derivador
float N = 10;
float tau=0.219;
float alfa=450.929;
float F=2;
float beta = 0;
float Taw = 0.9*h;
float bi;
float ad;
float bd;
float br;

// Variables P,I,D
float P;
float D;
float I;
float yant; 

double calcularZeta(double SP) {
    double lnSP = log(SP / 100.0);
    double numerador = fabs(lnSP);
    double denominador = sqrt(3.1416 * 3.1416 + lnSP * lnSP);
    return numerador / denominador;
}

void setup() {
    // iniciamos los perifericos (encoder, led, pwm)
    setupPeripherals();
    
    // Asi definimos la tarea de control, de la máxima prioridad en el nucleo 1
    xTaskCreatePinnedToCore(
            controlTask, // nombre de la rutina
            "simple proportional controller",
            8192,
            NULL,
            23, // prioridad de la tarea (0-24) , siendo 24 la prioridad más critica      
            NULL,
            CORE_1
    );  
    

    // ahora creamos una tarea para el menu de usuario
    xTaskCreatePinnedToCore(
        buttonTask, // nombre de la rutina
        "activate motor with the buttons",
        8192,
        NULL,
        10,  // definimos una prioridad baja para esta tarea
        NULL,
        CORE_0 // la vamos a ejecutar en el CORE_0 que es comparte tareas con el procesador, baja prioridad
    );
    

    // ponemos el led en verde para indicar que el control está activo
    setLedColor(0, 255, 0);
}


/* *************************************************************************
*                     FUNCION CRITICA DE CONTROL
***************************************************************************/ 


static void controlTask(void *pvParameters) {

    // Aqui configuro cada cuanto se repite la tarea

    const TickType_t taskInterval = 1000*h;  // repetimos la tarea cada tiempo de muestreo en milisegundos = 1000*0.01= 100ms
    


    // prototipo de una tarea repetitiva   
    for (;;) {
       TickType_t xLastWakeTime = xTaskGetTickCount(); 
       
       // leemos el cambio en el encoder y lo sumamos a la referencia
       reference +=  encoderKnob.getCount() * pulsesTodegreesKnob;  
       encoderKnob.clearCount(); //reseteamos el encoder
   
       // leemos el valor actual del ángulo
       y = encoderMotor.getCount() * pulsesTodegreesMotor;
       
       //calculamos el error       
       e = (beta*reference - y); 

       // Calculando el valor P
       z=calcularZeta(SP);
       wn=2.5/(tr);
       kp=(tau*wn*wn)*(2*z*F+1)/(alfa);
       ki=(F*tau*wn*wn*wn)/alfa;
       kd=((tau*wn)*(2*z+F)-1)/(alfa);

       bi= ki*h;
       ad = kd/(kd + N*h);
       bd = kd*N/(kd + N*h);
       br = h/Taw;
       //br=ki;
       

       P = kp*(e);

       // Calculando el valor D 
       D = ad*D - bd*(y-yant);

       // calculamos la accion de control proporcional
       u = P + I + D; 

       // realizamos la compensacion de zona muerta del motor u = u + deadzone * sign(u), si |u|>0
       u = compDeadZone(u, deadZone);

       // saturamos la señal de control para los limites energéticos disponibles entre umin=-5V y umax=5V
       usat = constrain(u, umin, umax);

       //enviamos la señal de control saturada al motor
       voltsToMotor(usat);     


       // ahora imprimimos para plotear al funcionamiento del controlador 
       printf(">Angulo:%.2f, Referencia:%0.2f, min:%d, max:%d, SP: %.2f \r\n", y, reference, -180, 180,SP);  
      
       // la tarea es crítica entonces esperamos exactamente taskInterval ms antes de activarla nuevamente
       vTaskDelayUntil(&xLastWakeTime, taskInterval);

       I = I + bi*(reference-y)+ br*(usat-u);
    //    I=constrain(I,-reference,reference);
       yant = y;
    }

}




// A continuacion creamos una tarea de menu de usuario


static void buttonTask(void *pvParameters) {
    
    for (;;) {
   
       // Si el boton con logo UN supera el umbral pongo un color naranja 
       if (touchRead(BUTTON_UN) > threshold){           
            
            // incrementamos  la referencia en 720°
            reference += 720;
            
            // cambiamos el color del led a azul por 1 segundo para indicar el cambio en la referencia
            setLedColor(0, 127, 255);
            vTaskDelay(2000);
            setLedColor(0, 255, 0);
            }

        // Si el boton con el dibujo de flor de la vida supera el umbral pongo un color azul claro 
       if (touchRead(BUTTON_LIFEFLOWER) > threshold){  
   
            // decrementamos  la referencia en 720°           
            reference-=720;
            // printf("Valor Sobrepico:%.2f, \r\n", SP);
            // cambiamos el color del led a rojo por 1 segundo para indicar el cambio en la referencia
            setLedColor(255, 0, 0);
            vTaskDelay(1000);
            setLedColor(0, 255, 0);
            }
       //   
       vTaskDelay(100); // espero 100 ms antes de hacer nuevamente la tarea
    }

}



void loop() {
    vTaskDelete(NULL);
}
