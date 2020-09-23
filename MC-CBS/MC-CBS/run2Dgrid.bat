@echo off 

set map="D:\ECBS_different_size\files\lak503d"
set algos="EPEA" "ICBS" "SYM" "ASYM" "MAX" "MAXpH" 
set output="D:\LA-MAPF\outputs\lak503dmap"
set time=60

for /l %%k in (5,5,25) do ( 
  for /l %%i in (0,1,49) do (
    echo Agent %%k ; Instance %%i
    for %%j in (%algos%) do (
      2DGrid.exe -m %map%.map  -a %map%map-%%kagents-size3to5-%%i.agents -o %output%-%%kagents-size3to5-%%j.csv -s %%j -l 2 -t %time%
    )
  )
)


pause


set map="D:\ECBS_different_size\files\10obs-20x20"
set algos="EPEA" "ICBS" "SYM" "ASYM" "MAX" "MAXpH" 
set output="D:\LA-MAPF\outputs\10obs-20x20map"
set time=60

for /l %%k in (2,1,8) do ( 
  for /l %%i in (0,1,49) do (
    echo Agent %%k ; Instance %%i
    for %%j in (%algos%) do (
      2DGrid.exe -m %map%.map  -a %map%map-%%kagents-size3-%%i.agents -o %output%-%%kagents-size3-%%j.csv -s %%j -l 2 -t %time%
    )
  )
)


pause


