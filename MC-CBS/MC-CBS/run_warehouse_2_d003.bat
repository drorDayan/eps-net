@echo off 

set algos="ICBS" "ASYM" "MAX" 
set output="outputs\warehouse_2_d003"
set time=300

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d003_multi_100_0.03_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d003_multi_100_0.03_grid.ymal" 

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


set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d003_multi_20_0.03_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d003_multi_20_0.03_grid.ymal" 

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


set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d003_multi_10_0.03_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d003_multi_10_0.03_grid.ymal" 

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



set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d003_multi_5_0.03_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d003_multi_5_0.03_grid.ymal" 

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

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d003_multi_3_0.03_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d003_multi_3_0.03_grid.ymal" 

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
