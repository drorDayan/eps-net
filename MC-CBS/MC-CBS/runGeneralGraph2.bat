@echo off 

set map="D:\Dev\eps-net\MC-CBS\instances\Dror\2 2D robots conflicts edge and vert.yml"
set algos="ASYM" "MAX" 
set output="outputs\2 2D rob conflicts double_cost_0.0000001_penalty"
set time=300

for /l %%k in (2,1,8) do (
  echo Agent=%%k
  for %%j in (%algos%) do (
    general-graph\x64\Release\generalgraph.exe -m %map% -o %output%.csv -s %%j -k %%k -l 1 -t %time%
  )
  general-graph\x64\Release\generalgraph.exe -m %map% -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
)

pause
