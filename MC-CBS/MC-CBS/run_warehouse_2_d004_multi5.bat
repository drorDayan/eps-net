@echo off 

set algos="MAX" 
set output="outputs\warehouse_2_d004_5"
set time=300000

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\warehouse_2_d004_easier_multi_100_0.04_eps_net.ymal" 

for %%m in (%maps%) do (
	for /l %%k in (2,1,5) do (
	  echo %%m
	  echo Agent=%%k
	  general-graph\x64\Release\generalgraph.exe -m %%m -o %output%.csv -s MAX -k %%k -l 1 -h 1 -t %time%
	)
)

pause
