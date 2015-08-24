echo Building ivcam sample app

g++ -I"../include" -I".." -I"." -O0 -g -Wall -c -fmessage-length=0 -MMD -MP -MF"ivcam_sample.d" -MT"ivcam_sample.d" -o "ivcam_sample.o" "ivcam_sample.cc"

g++ -o "ivcamsample" ./ivcam_sample.o -livcam


