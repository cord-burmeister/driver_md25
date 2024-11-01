#include <i2c_bus.hpp>
#include <driver/hiwonder.hpp>

hiwonder_driver::hiwonder_driver( ) {}
hiwonder_driver::~hiwonder_driver() {}


/*
                      FRONT                        
        +---------------------------------+        
        |           +---------+           |        
        |           |         |           |        
      +---------+   | 0x34    |   +---------+      
      | | 1     |   |         |   |     3 | |      
      | |       |   +--+---+--+   |       | |      
      +---------+      +---+      +---------+      
        |                                 |        
        |                                 |        
        |                                 |        
        |                                 |        
LEFT    |                                 |  RIGHT 
        |                                 |        
        |                                 |        
        |                                 |        
        |                                 |        
        |                                 |        
      +--------+                  +---------+      
      | | 2    |                  |     4 | |      
      | |      |                  |       | |      
      +--------+                  +---------+      
        |                                 |        
        |                                 |        
        |                                 |        
        |                                 |        
        +---------------------------------+        
                                                   
                       REAR                        
                                                   
*/

int hiwonder_driver::getDeviceIdFront () {
   return 52;
//   return 0x34;
}
