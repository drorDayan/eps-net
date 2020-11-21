@echo off 

set algos="MAX" 
set output="outputs\CBS1_v2_new"
set time=300000

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\123\multi_robot_CBS1_d004_v2_multi_100_0.04_eps_net.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (5,2,7) do (
	  echo %%m
	  echo Agent=%%k
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\123\multi_robot_CBS1_d004_v2_multi_25_0.04_eps_net.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (5,2,7) do (
	  echo %%m
	  echo Agent=%%k
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\123\multi_robot_CBS1_d004_v2_multi_20_0.04_eps_net.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (5,2,7) do (
	  echo %%m
	  echo Agent=%%k
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\123\multi_robot_CBS1_d004_v2_multi_10_0.04_eps_net.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (5,2,7) do (
	  echo %%m
	  echo Agent=%%k
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)
set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\123\multi_robot_CBS1_d004_v2_multi_5_0.04_eps_net.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (5,2,7) do (
	  echo %%m
	  echo Agent=%%k
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)
pause
