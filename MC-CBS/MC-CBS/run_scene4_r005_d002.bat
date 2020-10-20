@echo off 

set algos="MAX" 
set output="outputs\scene4_r005_d002"
set time=300

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\scene4_r005_d002_multi_100_0.02_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene4_r005_d002_multi_100_0.02_grid.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (2,2,6) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)


set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\scene4_r005_d002_multi_20_0.02_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene4_r005_d002_multi_20_0.02_grid.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (2,2,6) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)


set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\scene4_r005_d002_multi_10_0.02_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene4_r005_d002_multi_10_0.02_grid.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (2,2,6) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)



set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\scene4_r005_d002_multi_5_0.02_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene4_r005_d002_multi_5_0.02_grid.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (2,2,6) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\scene4_r005_d002_multi_3_0.02_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene4_r005_d002_multi_3_0.02_grid.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (2,2,6) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\scene4_r005_d002_multi_2_0.02_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene4_r005_d002_multi_2_0.02_grid.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (2,2,6) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)

pause
