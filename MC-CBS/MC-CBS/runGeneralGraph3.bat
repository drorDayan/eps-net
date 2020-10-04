@echo off 

set map="D:\Dev\eps-net\MC-CBS\instances\Dror\out1.ymal"
set algos="ASYM" "MAX" 
set output="outputs\pls work"
set time=300

for /l %%k in (2,1,2) do (
  echo Agent=%%k
  for %%j in (%algos%) do (
    general-graph\x64\Release\generalgraph.exe -m %map% -o %output%.csv -s %%j -k %%k -l 1 -t %time%
  )
  general-graph\x64\Release\generalgraph.exe -m %map% -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
)

pause
