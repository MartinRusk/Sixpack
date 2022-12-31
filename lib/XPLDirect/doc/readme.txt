Curiosity Workshop XPLDirect Arduino Library XPLDirect.h - Library for serial interface to Xplane SDK.
  
  Created by Michael Gerlicher, Curiosity Workshop,  September 2020.

  To report problems, download updates and examples, suggest enhancements or get technical support, please visit out patreon page or our YouTube Page:
     
     www.patreon.com/curiosityworkshop
     https://www.youtube.com/channel/UCISdHdJIundC-OSVAEPzQIQ

  Branded versions of the plugin are available for commercial redistribution, contact me for details.

  Please support this project in one of the following ways:

        Free:  like and subscribe to our youtube channel!
        Free:  If you mine crypto using unmineable, use referral code 706z-n61d.  We get a small cut and you get reduced fees.  Win-Win!

        Support us at patreon.com/curiosityworkshop
        
        Donate crypto:
            Cardano ADA:  addr1qypp08v6kqsjj9mexutsa9waltm7dp0seqy44nkpwwgphhwggftvsnlz3pwvhf2m3yg0ssjj3fwvcdep0e5u8jdssrpsmqeccx
            Bitcoin:      3EF6dBEWJcwSppCqPxoGbPnGyFjdq23pb1
            Etherium:     0x3c5fc25baedd6220a9c537d0c7a9adb2b2ba235b
            Litecoin:     M8oxczdoAeE9JpXfPRfPskehokwoiEZ5UF


  This version of the arduino library should be run with plugin build 2106171 or above.

** 12/17/2021 build 2112161
              - Plugin updated to register devices after a plane completely loads.  Some aircraft have custom datarefs 
		associated with them that won't be available until then.

	      - updated menu options to include the ability to log serial data.  Defaults to off and the settings are saved for 
		next launch.  
		
	      - just a note, xplane does not support multiple command activations during one cycle, so
                    int XPLDirect::commandTrigger(int commandHandle, int triggerCount)
		triggerCount causes only one iteration of a command to occur regardless of the number.  I am exploring options
		to deal with this issue.  Let me know if it is important to you.




 ** 6/10/2021 - new function added:  sendResetRequest();
                    eg: Xinterface.sendResetRequest();
                    This causes the plugin to reset and re-query the Xplane/Direct arduino devices.  
 
 ** 5/28/2021  - the plugin now responds to messages from xplane when aircraft are unloaded and reloaded.  This will ask all the XPL/Direct 
                  devices to reset and will re-register all datarefs and commands from them.  This allows for one sketch to work with multiple
                  aircraft, particularly when the aircraft come with custom datarefs.

  ** 5/27/2021  - fixed timeout problem when attempting to register a dataref that doesnt exist.  The registration count will still increase but 
                  processing for the invalid dataref will not occur.  This is useful if you have a sketch that should work with multiple aircraft,
                  some that register custom datarefs and some that don't.

 ** 5/14/2021   - added support for int array datarefs.



 
 Previous Releases:


** 02/01/2021   - Implemented divider for float data being sent to plugin.  
                - Changed format of floats sent to plugin to reduce dataflow
                - Increased buffer size to accomodate string datarefs up to 190 bytes long.
                - plugin menu items changed for disengage/re-engage to only one item that changes depending on status
                - implementation of data types (strings usually).  This only works with data coming from xplane.  It is up to
                    you to confirm your data buffer is large enough to accomodate the length of the strings that you are receiving.
                    Also, the data can't exceed the maximum buffer size -5 (195 bytes)
                - addition of methods hasUpdated(handle) and datarefsUpdated().  
                    int hasUpdated(int handle);     //  returns true if the specified dataref has changed since the last call to hasUpdated().  
                    int datarefsUpdated();          //  returns true if any datarefs have been updated since the last call to datarefsUpdated().
                - update both the plugin and the library for full functionality
** 12/19/2020   - Added support for commandBegin and commandEnd.  Avoid using these unless you are sure of how they work. 
                  commandBegin commands must be balanced with commandEnd or the world will start to wobble.  
                  This can be useful on the poor implentation of commands and datarefs by a notoriously popular 737 simulator plugin.

                  Usage:    commandStart(myCommand);            // triggers a command to begin
                            commandEnd(myCommand);              // ... then end
                
                - And a reminder, if you are defining a command using a variable, use an "int" type, not "long int" or it won't work 
                  quite right.   

                  int myCommand;                    // will be the handle to the command for triggers/begin/end
                  int myCommandVariable;            // will be the variable the library watches for changes to trigger commands.

                  myCommand = registerCommand("sim/radios/nav2_standy_flip", &myCommandVariable);


                - Support added for version control.  The plugin can query the arduino device for the version of the library it was 
                  compiled with to flag incompatibilities.  

                - corrected type on "int" sends to the plugin to be "long int"
                
** 12/16/2020   - Minor bug fixes

** 11/23/2020   - Added support for manual triggering of registered commands.  

                    an function overload was added for command registration if you only plan to trigger it manually:
                    
                            int registerCommand(char *commandName);     // returns handle to command or negative number on error

                    Two new functions were added:

                            int commandTrigger();                   // triggers the command one time
                            int commandTrigger(int triggerCount);   // triggers the command triggerCount times

                    Example:

                        setup
                            int myCommand = Xinterface.registerCommand("sim/radios/nav2_standy_flip", 6);

                        loop

                            Xinterface.commandTrigger(myCommand);

                This could be useful if you need to trigger commands with a rotary encoder or similar device.

                - small change made to reduce data flow when sending integers to the plugin.

** 11/21/2020   - So I put a serial sniffer on the line to monitor communications between the plugin and the library and found a couple places
                  where packets were being sent twice from the plugin, causing potential buffer overflows and other quirks.  
                  I took the opportunity to revamp the rx engine on the library to increase its robustness.  Please update the library and the 
                  plugin to this version, it has tested quite stable by multiple users.  I have a couple other things on my to-do list but I don't 
                  foresee any short-term major changes at this point.  Thank you for your patience!

** 11/20/2020   - Fixed a minor bug with command registration

                - Added support for PROGMEM.  On some boards, this will prevent the names of the datarefs from being transfered into usable RAM.
                  When *many* datarefs are being registered this makes a substantial difference with memory usage.  Some boards (SAM stuff) don't work
                  this way so the #define XPL_USE_PROGMEM will need to be disabled if you have compiler errors, particularly with things like strlen_PF
                  commands in the library.  Disable the #define and all will be better.  As I prefer more elegant solutions, I will work on making
                  this automatic in the future.
                  
                  To use the change, you will need to wrap your dataref registration command names with an F() macro, for example:

                  Xinterface.registerDataRef(F("sim/cockpit/electrical/beacon_lights_on"), XPL_WRITE, 100, 0, &beacon);

                  For now you can use either, but if you are registering many datarefs you will need to use this on older boards.

                - Minor changes made to the plugin, please update it as well.  


** 11/17/2020    - Added registerCommand overload: registerCommand(char *commandname, int datapin);  
                    This links an arduino datapin to a command, just specify which pin and we do the rest of the work!
                 - fixed a minor bug with command registrations

** 11/16/2020    - changed when dataref and command registration takes place.  Xplane loads plugins, then loads aircraft.  Some 
                    aircraft create new datarefs which would not be found by this plugin.  Registration of datarefs now occurs on the 
                    first flight loop.  If I can find a better way to implement this it will be revised.

                 - variable length loop times implemented for busy loops or slower processors.

                        setMinimumLoopTimes(long int loopTimeBeforeRegistration, long int loopTimeAfterRegistration)
                
                    Add this after the "begin()" command to change the time that xloop watches for serial input before returning.

                    "loopTimeBeforeRegistration" is in milliseconds and xloop will stay looping for this time until all datarefs are
                    registered.  This may need to be higher if the plugin is having difficulty with dataref registrations because of 
                    high packet lengths during the registration time, especially if your code is busier.  The default is 1000ms.

                    "loopTimeAfterRegistration" is also in milliseconds and xloop will stay looping for this time after the dataref 
                    registration process is complete.  This can be a smaller value unless your application is receiving large volumes
                    of data from the plugin.  The default is 20ms.

                - removal of all use of String types
                - streamlining of send and receive buffers
               
** 11/08/2020:  - Memory space allocation for datarefs and xplane commands is now dynamic.  This can be good and bad, as it does not
                  watch for low memory situations.  If things begin to act erratically, memory may be the issue.  
                - Support for Xplane commands has now been implemented.
			            Register a command:  registerCommand(command name, *value);   See example XPLDirectCommandDemo for an example.
                - You will need to update the plugin also with this revision.

** 11/02/2020:  - Changes made to frame formatting to save memory, processing time and data stream traffic.
				- Enabled float type for divider parameter when registering datarefs.  
				  For example, now we can assign a divider of .1 if we only want one decimal precision.
                - Problem fixed with data collisions if arduino board begins sending data before all datarefs are registered
                  with the plugin.
                - Fixed a problem sending negative floats from the arduino board since some don't support floats to string buffers.  
                  What a pain!
				- You will need to update the plugin also with this revision.

** 10/31/2020:  Update to correct an issue with registration of array type datarefs.
** 10/27/2020:  Initial Release 10/27/2020


Installation
-------------
Extract the XPLDirect folder and place it into the libraries folder of your arduino sketches


Usage
------


Include the XPLDirect library:

     #include <XPLDirect.h> 

Create an instance, supply the pointer to your serial port:
     
	 XPLDirect Xinterface(&Serial);

Assign variables for the datarefs you plan to register.  X-plane considers int type to include long int.

     long int beacon;

In your 'begin' code section, initialize your serial port and also the interface:

 Serial.begin(XPLDIRECT_BAUDRATE); 
 Xinterface.begin("Xplane Demo");       
 //Xinterface.setMinimumLoopTimes(beforeRegistration, afterRegistration);  // only needed for specific applications.  Defaults are 1000 and 20.

Also in your 'begin' code, register the datarefs you want to use.
   parameters:  

      char * dataref name, the x-plane dataref you wish to register
      int    write type:				XPL_READ to obtain the value of the dataref from xplane
										XPL_WRITE to send the value to xplane
                                      XPL_READWRITE to do both (not really recommended)
      int    update frequency (ms)	how often we should update this dataref, to reduce traffic
      float  update divider			this reduces the precision of data transferred back and forth.  For example,
                                      if you have a tachometer and only need 10rpm resolution, put 10 here.  This
										will reduce the traffic sent between the arduino and the xplane plugin.  
      long int * variable				the variable containing data that will be written to / read from xplane.


 Xinterface.registerDataRef("sim/cockpit/electrical/beacon_lights_on", XPL_READ, 100, 0, &beacon); 

In your 'loop' code, run the xloop routine:
 
 Xinterface.xloop(); 

That's it!  in this example, the variable 'beacon' will automatically update to reflect the status of the beacon lights in x-plane.

registerDataRef is overloaded also with a float version:

     int registerDataRef(const char*, int, unsigned int, int, float*);     

registerDataRef also includes overloads to handle array type datarefs.  The last parameter is the array element to use:

    int registerDataRef(const char*, int, unsigned int, int, long int*, int);       
    int registerDataRef(const char*, int, unsigned int, int, float*, int);           


-- Avoid using delays in your loop as it can create buffer overflows that will cause frame loss.

--Boards with limited memory such as the original uno and nano fill up quickly and can only handle a few datarefs, depending on 
  the additional code in your sketch.  If you are experiencing erratic operation, try reducing the number of datarefs and commands
  you are using.  If you need more datarefs and your board can handle it, modify the header as needed: 
  
  #define XPLDIRECT_MAXDATAREFS_ARDUINO 10
  #define XPLDIRECT_MAXCOMMANDS_ARDUINO 10

  and specify the number of datarefs/commands you are using.  

