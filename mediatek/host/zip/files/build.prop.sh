#!/system/bin/sh

FILE=/system/build.prop
TMPFILE=$FILE.tmp

line1=ro.build.selinux

line1Arg=1

lineNum=

prop=$line1     
arg=$line1Arg   
if grep -Fq $prop $FILE ; then  
lineNum=`sed -n "/${prop}/=" $FILE`   
echo $lineNum
sed -i "${lineNum} c${prop}=${arg}" $FILE   
else
echo "$prop does not exist in build.prop"   
echo "appending to end of build.prop"
echo $prop=$arg >> $FILE
fi
