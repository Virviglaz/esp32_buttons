# ESP32 buttons manager

Description: This library allows you to dynamically or statically add a button handling functionality to your project just in a few lines of code.
It creates a service task in the background that configures the pin of ESP32 gpio port and keeps polling it for activity.
If the activity were detected the related callback function is being called.

## Example of use
### Callback function
~~~cpp
static void btn_handler(void *param)
{
  char *text = (char *)param;

  printf("%s\n", text);
}
~~~
### Code example (static usage)
~~~cpp
#include "esp_buttons.h"

buttons btn;
btn.add(GPIO_NUM_13, btn_handler, POSEDGE, (void *)"BUTTON 1 PIN 13 PRESSED");
btn.add(GPIO_NUM_12, btn_handler, NEGEDGE, (void *)"BUTTON 2 PIN 12 PRESSED");
btn.add(GPIO_NUM_14, btn_handler, NEGEDGE, (void *)"BUTTON 3 PIN 14 PRESSED");
~~~
### Code example (dynamic usage)
~~~cpp
buttons *btn = new buttons();
btn->add(GPIO_NUM_13, btn_handler, POSEDGE, (void *)"BUTTON 1 PIN 13");
btn->add(GPIO_NUM_12, btn_handler, NEGEDGE, (void *)"BUTTON 1 PIN 12");
btn->add(GPIO_NUM_14, btn_handler, NEGEDGE, (void *)"BUTTON 1 PIN 14");
~~~
### Release the resources
~~~cpp
delete(btn);
~~~
### Port monitor output:
##### BUTTON 3 PIN 14 PRESSED
##### BUTTON 3 PIN 14 PRESSED
##### BUTTON 3 PIN 14 PRESSED
##### BUTTON 1 PIN 13 PRESSED
##### BUTTON 2 PIN 12 PRESSED
