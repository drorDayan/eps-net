@echo off 

set map="D:\Dev\CBS\MC-CBS\instances\drones\roadmapWithConflicts_swap50.yaml"
set algos="ICBS" "ASYM" "MAX" 
set output="outputs\some_file_name"
set time=300

for /l %%k in (2,1,8) do (
  echo Agent=%%k
  for %%j in (%algos%) do (
    general-graph\x64\Release\generalgraph.exe -m %map% -o %output%.csv -s %%j -k %%k -l 1 -t %time%
  )
  general-graph\x64\Release\generalgraph.exe -m %map% -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
)

pause