#pragma once

char *e404_html = R"(
<!DOCTYPE html>
<html>
 <head>
  <meta charset="utf-8">
  <title>Microbial fuel cell</title>
 </head>
 <body>
<h1 style="text-align:center; font-size:80px">404</h1>	
<h1 style="text-align:center;font-size:30px">Page Not Found</h1>
<h1 style="text-align:center;">The Page you are looking for doesn't exist or an other error occured</h1>	
 </body>
</html>)";

char *hello_world_page = R"(<h1>Hello world!</h1>)";

char *table_page_start
	= R"(<html><head><title>Microbial fuel cell</title></head><body><h1>Data table</h1><table border="1"><tr><th>Sensor ID</th><th>Hour</th><th>Minute</th><th>Date</th><th>Month</th><th>CO2 [ppm]</th><th>Temp [C]</th><th>Humid [%]</th></tr>)";
char *table_page_end = R"(</table></body></html>)";
