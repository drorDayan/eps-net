@echo off 

set algos="ICBS" "ASYM" "MAX" 
set output="outputs\warehouse_2_d004"
set time=300

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d004_multi_100_0.04_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d004_multi_100_0.04_grid.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (2,1,3) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)


set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d004_multi_20_0.04_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d004_multi_20_0.04_grid.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (2,1,3) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)


set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d004_multi_10_0.04_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d004_multi_10_0.04_grid.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (2,1,3) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)



set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d004_multi_5_0.04_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d004_multi_5_0.04_grid.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (2,1,3) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d004_multi_3_0.04_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d004_multi_3_0.04_grid.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (2,1,3) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)


pause
