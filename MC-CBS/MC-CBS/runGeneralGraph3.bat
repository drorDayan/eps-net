@echo off 

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\multi10_0.05_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\multi10_0.05_grid.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\multi10_0.05_random1.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\multi10_0.05_random2.ymal"
set algos="ASYM" "MAX" 
set output="outputs\pls work"
set time=300

for %%m in (%maps%) do (
	for /l %%k in (2,1,2) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\multi100_0.05_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\multi100_0.05_grid.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\multi100_0.05_random.ymal"
set algos="ASYM" "MAX" 
set output="outputs\pls work"
set time=300

for %%m in (%maps%) do (
	for /l %%k in (2,1,2) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\multi100000_0.05_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\multi100000_0.05_grid.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\multi100000_0.05_random.ymal"
set algos="ASYM" "MAX" 
set output="outputs\pls work"
set time=300

for %%m in (%maps%) do (
	for /l %%k in (2,1,2) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)


pause
