#include <i2c_bus.hpp>
#include <driver/md25.hpp>

md25_driver::md25_driver( ) {}
md25_driver::~md25_driver() {}


/*
                      FRONT                        
        +---------------------------------+        
        |           +---------+           |        
        |           |         |           |        
      +---------+   | 0x58    |   +---------+      
      | | 2     |   |         |   |     1 | |      
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
      +--------+      +---+       +---------+      
      | | 1    |   +--+---+--+    |     2 | |      
      | |      |   |         |    |       | |      
      +--------+   |  0x5A   |    +---------+      
        |          |         |            |        
        |          +---------+            |        
        |                                 |        
        |                                 |        
        +---------------------------------+        
                                                   
                       REAR                        
                                                   
*/


int md25_driver::getDeviceIdFront ()
{
  return deviceIdFront;
}

int md25_driver::getDeviceIdRear ()
{
  return deviceIdRear;
}

  int md25_driver::getFrontLeftEncoderId ()
  {
    return 2;
  }
  int md25_driver::getFrontRightEncoderId ()
  {
    return 1;
  }
  int md25_driver::getRearLeftEncoderId ()
  {
    return 1;
  }
  int md25_driver::getRearRightEncoderId ()
  {
    return 2;
  }

