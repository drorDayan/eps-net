@echo off 

set algos=
set output="outputs\CBS1_v3_obs1"
set time=300000

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\v3\multi_robot_CBS1_d004_v3_obs_multi_50_0.04_eps_net.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (5,2,7) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )

	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\v3\multi_robot_CBS1_d004_v3_obs_multi_25_0.04_eps_net.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (5,2,7) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\v3\multi_robot_CBS1_d004_v3_obs_multi_10_0.04_eps_net.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (5,2,7) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\v3\multi_robot_CBS1_d004_v3_obs_multi_15_0.04_eps_net.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (5,2,7) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\v3\multi_robot_CBS1_d004_v3_obs_multi_5_0.04_eps_net.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (5,2,7) do (
	  echo %%m
	  echo Agent=%%k
	  for %%j in (%algos%) do (
		general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s %%j -k %%k -l 1 -t %time%
	  )
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)
pause
