
* Overview

The MTConnect adapter is toolkit is a framework for quickly achieving MTConnect 
compliance. The framework is organized as follows. 

src               - Common framework library
src/adapter       - Abstract adabter
src/client        - Client of the adapter
src/device_datum  - Classes that manage the individual pieces of data retrieved from 
                    a machine tool.
src/server        - Socket serevr
src/string_buffer - A auto-extending text buffer that can be time-stamped

fake              - Minimal adapter with only power. Makefile for UNIX
fake/fake.cpp     - main for the application
fake/fake_adapter - The adapter

fanuc             - FANUC adapter (minimal for now, needs work). Include MSDev SLN
fanuc/FanucAdapter - main for the application
fanuc/fanuc_adapter - The adapter for the fanuc controller

lib               - Future location for the framework library

include           - Future location for the framework include files

emc               - A complete implementation of the emc2 linux CNC simulator adapter
emc/emc.cpp       - main for the application
emc/emc_adapter   - The adapter

* Details

  The framework provides all the socket communications to multiple clients. The 
  abstract adapter provide everything except for the connectivity to the 
  propriatary API.
  
* To Do
  
  Complete FANUC
  Port to .NET
  Start OPC
  Start Okuma
  
