@echo off 

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi1_0.05_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi1_0.05_grid.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi1_0.05_random.ymal"  
set algos="ICBS" "ASYM" "MAX" 
set output="outputs\scene3_r001_d005"
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

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi3_0.05_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi3_0.05_grid.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi3_0.05_random.ymal"  

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

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi5_0.05_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi5_0.05_grid.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi5_0.05_random.ymal"  

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

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi10_0.05_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi10_0.05_grid.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi10_0.05_random.ymal"  

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

set maps="D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi100_0.05_eps_net.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi100_0.05_grid.ymal" "D:\Dev\eps-net\MC-CBS\instances\Dror\scene3_r001_d005_multi100_0.05_random.ymal"  

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
