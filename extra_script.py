import datetime

x = datetime.datetime.now()
versionFW = f"{x.strftime('%Y')+x.strftime('%m')+x.strftime('%d')+x.strftime('%H')+x.strftime('%M')+x.strftime('%S')}"

fw = open('.pio/build/esp32doit-devkit-v1/VERSION.html','w')
print(versionFW, file=fw)
fw.close()

f = open('include/version.h','w')
print('#define actualVersion ', versionFW, file=f)
f.close()
