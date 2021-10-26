#!/bin/bash
#pio project init -b megaatmega2560
#pio run -e megaatmega2560 --target upload
#picocom -b 115200 /dev/ttyACM0
BRD=megaatmega2560
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
# endfile

