#!/bin/bash

BRD=featheresp32
PS3="Select option for ${BRD}..."

select opt in Init Upload Quit; do

case $opt in
   Init)
     echo "Init..."
     pio project init -b ${BRD}
     ;;

   Upload)
     echo "Upload..."
     pio run -e ${BRD} --target upload
     ;;

   Quit)
      echo "Goodbye..."
      break;
      ;;
      *)
   esac

done

#picocom -b 115200 /dev/ttyUSB0  .terminal example
# endfile

