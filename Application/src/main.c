/**
  ******************************************************************************
  * @file    main.c 
  * @author  Vu Trung Hieu
  * @version V1.0
  * @date    13-January-2018
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */  

/* Includes ------------------------------------------------------------------*/
#include "include.h"

/** @addtogroup GIMBAL_BLDC_NCKH
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SVPWM_TABLE_SIZE    6480
#define SPEED_CONVERT       46296.296296296300f//100000/SVPWM_TABLE_SIZE/6*1000
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern volatile uint32_t tick_count;
extern volatile uint32_t u32_tick_cnt_0, u32_tick_cnt_1;

static const uint16_t u16_svpwm_table[SVPWM_TABLE_SIZE] = 
{134,133,133,132,132,131,131,131,130,130,129,129,128,128,127,127,126,126,125,125,124,124,123,123,123,122,122,121,121,120,120,119,119,118,118,117,117,117,116,116,115,115,114,114,113,113,112,112,112,111,111,110,110,109,109,108,108,108,107,107,106,106,105,105,105,104,104,103,103,102,102,102,101,101,100,100,99,99,99,98,98,97,97,97,96,96,95,95,94,94,94,93,93,92,92,92,91,91,90,90,90,89,89,88,88,88,87,87,86,86,86,85,85,84,84,84,83,83,83,82,82,81,81,81,80,80,79,79,79,78,78,78,77,77,76,76,76,75,75,75,74,74,74,73,73,72,72,72,71,71,71,70,70,70,69,69,68,68,68,67,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,62,62,61,61,61,60,60,60,59,59,59,58,58,58,57,57,57,56,56,56,55,55,55,54,54,54,54,53,53,53,52,52,52,51,51,51,50,50,50,50,49,49,49,48,48,48,47,47,47,47,46,46,46,45,45,45,45,44,44,44,43,43,43,43,42,42,42,41,41,41,41,40,40,40,40,39,39,39,38,38,38,38,37,37,37,37,36,36,36,36,35,35,35,35,34,34,34,34,33,33,33,33,32,32,32,32,31,31,31,31,30,30,30,30,29,29,29,29,29,28,28,28,28,27,27,27,27,26,26,26,26,26,25,25,25,25,25,24,24,24,24,23,23,23,23,23,22,22,22,22,22,21,21,21,21,21,20,20,20,20,20,19,19,19,19,19,19,18,18,18,18,18,17,17,17,17,17,17,16,16,16,16,16,16,15,15,15,15,15,15,14,14,14,14,14,14,13,13,13,13,13,13,12,12,12,12,12,12,12,11,11,11,11,11,11,11,10,10,10,10,10,10,10,9,9,9,9,9,9,9,9,8,8,8,8,8,8,8,8,7,7,7,7,7,7,7,7,7,6,6,6,6,6,6,6,6,6,5,5,5,5,5,5,5,5,5,5,5,4,4,4,4,4,4,4,4,4,4,4,3,3,3,3,3,3,3,3,3,3,3,3,3,3,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9,10,10,10,10,10,10,10,11,11,11,11,11,11,11,12,12,12,12,12,12,12,13,13,13,13,13,13,14,14,14,14,14,14,15,15,15,15,15,15,16,16,16,16,16,16,17,17,17,17,17,17,18,18,18,18,18,19,19,19,19,19,19,20,20,20,20,20,21,21,21,21,21,22,22,22,22,22,23,23,23,23,23,24,24,24,24,25,25,25,25,25,26,26,26,26,26,27,27,27,27,28,28,28,28,29,29,29,29,29,30,30,30,30,31,31,31,31,32,32,32,32,33,33,33,33,34,34,34,34,35,35,35,35,36,36,36,36,37,37,37,37,38,38,38,38,39,39,39,40,40,40,40,41,41,41,41,42,42,42,43,43,43,43,44,44,44,45,45,45,45,46,46,46,47,47,47,47,48,48,48,49,49,49,50,50,50,50,51,51,51,52,52,52,53,53,53,54,54,54,54,55,55,55,56,56,56,57,57,57,58,58,58,59,59,59,60,60,60,61,61,61,62,62,62,63,63,63,64,64,64,65,65,65,66,66,66,67,67,67,68,68,68,69,69,70,70,70,71,71,71,72,72,72,73,73,74,74,74,75,75,75,76,76,76,77,77,78,78,78,79,79,79,80,80,81,81,81,82,82,83,83,83,84,84,84,85,85,86,86,86,87,87,88,88,88,89,89,90,90,90,91,91,92,92,92,93,93,94,94,94,95,95,96,96,97,97,97,98,98,99,99,99,100,100,101,101,102,102,102,103,103,104,104,105,105,105,106,106,107,107,108,108,108,109,109,110,110,111,111,112,112,112,113,113,114,114,115,115,116,116,117,117,117,118,118,119,119,120,120,121,121,122,122,123,123,123,124,124,125,125,126,126,127,127,128,128,129,129,130,130,131,131,131,132,132,133,133,134,135,137,138,140,141,143,144,146,147,148,150,151,153,154,156,157,159,160,162,163,165,166,168,169,171,172,173,175,176,178,179,181,182,184,185,187,188,190,191,193,194,196,197,199,200,202,203,205,206,208,209,211,212,214,215,217,218,220,221,223,224,226,227,229,230,232,233,235,236,238,239,241,242,244,245,247,248,250,251,253,254,256,257,259,260,262,263,265,266,268,269,271,272,274,275,277,279,280,282,283,285,286,288,289,291,292,294,295,297,298,300,302,303,305,306,308,309,311,312,314,315,317,318,320,322,323,325,326,328,329,331,332,334,335,337,339,340,342,343,345,346,348,349,351,353,354,356,357,359,360,362,363,365,367,368,370,371,373,374,376,378,379,381,382,384,385,387,389,390,392,393,395,396,398,400,401,403,404,406,407,409,411,412,414,415,417,418,420,422,423,425,426,428,430,431,433,434,436,437,439,441,442,444,445,447,449,450,452,453,455,457,458,460,461,463,465,466,468,469,471,473,474,476,477,479,481,482,484,485,487,489,490,492,493,495,497,498,500,501,503,505,506,508,509,511,513,514,516,517,519,521,522,524,526,527,529,530,532,534,535,537,538,540,542,543,545,547,548,550,551,553,555,556,558,560,561,563,564,566,568,569,571,573,574,576,577,579,581,582,584,586,587,589,590,592,594,595,597,599,600,602,604,605,607,608,610,612,613,615,617,618,620,622,623,625,626,628,630,631,633,635,636,638,640,641,643,644,646,648,649,651,653,654,656,658,659,661,663,664,666,668,669,671,672,674,676,677,679,681,682,684,686,687,689,691,692,694,696,697,699,701,702,704,705,707,709,710,712,714,715,717,719,720,722,724,725,727,729,730,732,734,735,737,739,740,742,744,745,747,749,750,752,754,755,757,759,760,762,764,765,767,769,770,772,774,775,777,779,780,782,784,785,787,789,790,792,794,795,797,799,800,802,804,805,807,809,810,812,814,815,817,819,820,822,824,825,827,829,830,832,834,835,837,839,840,842,844,845,847,849,850,852,854,855,857,859,860,862,864,865,867,869,870,872,874,875,877,879,880,882,884,885,887,889,890,892,894,895,897,899,901,902,904,906,907,909,911,912,914,916,917,919,921,922,924,926,927,929,931,932,934,936,937,939,941,942,944,946,947,949,951,953,954,956,958,959,961,963,964,966,968,969,971,973,974,976,978,979,981,983,984,986,988,989,991,993,994,996,998,999,1001,1003,1005,1006,1008,1010,1011,1013,1015,1016,1018,1020,1021,1023,1025,1026,1028,1030,1031,1033,1035,1036,1038,1040,1041,1043,1045,1046,1048,1050,1052,1053,1055,1057,1058,1060,1062,1063,1065,1067,1068,1070,1072,1073,1075,1077,1078,1080,1082,1083,1085,1087,1088,1090,1092,1093,1095,1097,1098,1100,1102,1104,1105,1107,1109,1110,1112,1114,1115,1117,1119,1120,1122,1124,1125,1127,1129,1130,1132,1134,1135,1137,1139,1140,1142,1144,1145,1147,1149,1150,1152,1154,1155,1157,1159,1160,1162,1164,1165,1167,1169,1170,1172,1174,1175,1177,1179,1180,1182,1184,1185,1187,1189,1190,1192,1194,1195,1197,1199,1200,1202,1204,1205,1207,1209,1210,1212,1214,1215,1217,1219,1220,1222,1224,1225,1227,1229,1230,1232,1234,1235,1237,1239,1240,1242,1244,1245,1247,1249,1250,1252,1254,1255,1257,1259,1260,1262,1264,1265,1267,1269,1270,1272,1274,1275,1277,1279,1280,1282,1284,1285,1287,1289,1290,1292,1294,1295,1297,1298,1300,1302,1303,1305,1307,1308,1310,1312,1313,1315,1317,1318,1320,1322,1323,1325,1327,1328,1330,1331,1333,1335,1336,1338,1340,1341,1343,1345,1346,1348,1350,1351,1353,1355,1356,1358,1359,1361,1363,1364,1366,1368,1369,1371,1373,1374,1376,1377,1379,1381,1382,1384,1386,1387,1389,1391,1392,1394,1395,1397,1399,1400,1402,1404,1405,1407,1409,1410,1412,1413,1415,1417,1418,1420,1422,1423,1425,1426,1428,1430,1431,1433,1435,1436,1438,1439,1441,1443,1444,1446,1448,1449,1451,1452,1454,1456,1457,1459,1461,1462,1464,1465,1467,1469,1470,1472,1473,1475,1477,1478,1480,1482,1483,1485,1486,1488,1490,1491,1493,1494,1496,1498,1499,1501,1502,1504,1506,1507,1509,1510,1512,1514,1515,1517,1518,1520,1522,1523,1525,1526,1528,1530,1531,1533,1534,1536,1538,1539,1541,1542,1544,1546,1547,1549,1550,1552,1554,1555,1557,1558,1560,1562,1563,1565,1566,1568,1569,1571,1573,1574,1576,1577,1579,1581,1582,1584,1585,1587,1588,1590,1592,1593,1595,1596,1598,1599,1601,1603,1604,1606,1607,1609,1610,1612,1614,1615,1617,1618,1620,1621,1623,1625,1626,1628,1629,1631,1632,1634,1636,1637,1639,1640,1642,1643,1645,1646,1648,1650,1651,1653,1654,1656,1657,1659,1660,1662,1664,1665,1667,1668,1670,1671,1673,1674,1676,1677,1679,1681,1682,1684,1685,1687,1688,1690,1691,1693,1694,1696,1697,1699,1701,1702,1704,1705,1707,1708,1710,1711,1713,1714,1716,1717,1719,1720,1722,1724,1725,1727,1728,1730,1731,1733,1734,1736,1737,1739,1740,1742,1743,1745,1746,1748,1749,1751,1752,1754,1755,1757,1758,1760,1761,1763,1764,1766,1767,1769,1770,1772,1773,1775,1776,1778,1779,1781,1782,1784,1785,1787,1788,1790,1791,1793,1794,1796,1797,1799,1800,1802,1803,1805,1806,1808,1809,1811,1812,1814,1815,1817,1818,1820,1821,1823,1824,1826,1827,1828,1830,1831,1833,1834,1836,1837,1839,1840,1842,1843,1845,1846,1848,1849,1851,1852,1853,1855,1856,1858,1859,1861,1862,1864,1865,1866,1866,1867,1867,1868,1868,1868,1869,1869,1870,1870,1871,1871,1872,1872,1873,1873,1874,1874,1875,1875,1876,1876,1876,1877,1877,1878,1878,1879,1879,1880,1880,1881,1881,1882,1882,1882,1883,1883,1884,1884,1885,1885,1886,1886,1887,1887,1887,1888,1888,1889,1889,1890,1890,1891,1891,1891,1892,1892,1893,1893,1894,1894,1894,1895,1895,1896,1896,1897,1897,1897,1898,1898,1899,1899,1900,1900,1900,1901,1901,1902,1902,1902,1903,1903,1904,1904,1905,1905,1905,1906,1906,1907,1907,1907,1908,1908,1909,1909,1909,1910,1910,1911,1911,1911,1912,1912,1913,1913,1913,1914,1914,1915,1915,1915,1916,1916,1916,1917,1917,1918,1918,1918,1919,1919,1920,1920,1920,1921,1921,1921,1922,1922,1923,1923,1923,1924,1924,1924,1925,1925,1925,1926,1926,1927,1927,1927,1928,1928,1928,1929,1929,1929,1930,1930,1931,1931,1931,1932,1932,1932,1933,1933,1933,1934,1934,1934,1935,1935,1935,1936,1936,1936,1937,1937,1937,1938,1938,1938,1939,1939,1939,1940,1940,1940,1941,1941,1941,1942,1942,1942,1943,1943,1943,1944,1944,1944,1945,1945,1945,1945,1946,1946,1946,1947,1947,1947,1948,1948,1948,1949,1949,1949,1949,1950,1950,1950,1951,1951,1951,1952,1952,1952,1952,1953,1953,1953,1954,1954,1954,1954,1955,1955,1955,1956,1956,1956,1956,1957,1957,1957,1958,1958,1958,1958,1959,1959,1959,1959,1960,1960,1960,1961,1961,1961,1961,1962,1962,1962,1962,1963,1963,1963,1963,1964,1964,1964,1964,1965,1965,1965,1965,1966,1966,1966,1966,1967,1967,1967,1967,1968,1968,1968,1968,1969,1969,1969,1969,1970,1970,1970,1970,1970,1971,1971,1971,1971,1972,1972,1972,1972,1973,1973,1973,1973,1973,1974,1974,1974,1974,1974,1975,1975,1975,1975,1976,1976,1976,1976,1976,1977,1977,1977,1977,1977,1978,1978,1978,1978,1978,1979,1979,1979,1979,1979,1980,1980,1980,1980,1980,1980,1981,1981,1981,1981,1981,1982,1982,1982,1982,1982,1982,1983,1983,1983,1983,1983,1983,1984,1984,1984,1984,1984,1984,1985,1985,1985,1985,1985,1985,1986,1986,1986,1986,1986,1986,1987,1987,1987,1987,1987,1987,1987,1988,1988,1988,1988,1988,1988,1988,1989,1989,1989,1989,1989,1989,1989,1990,1990,1990,1990,1990,1990,1990,1990,1991,1991,1991,1991,1991,1991,1991,1991,1992,1992,1992,1992,1992,1992,1992,1992,1992,1993,1993,1993,1993,1993,1993,1993,1993,1993,1994,1994,1994,1994,1994,1994,1994,1994,1994,1994,1994,1995,1995,1995,1995,1995,1995,1995,1995,1995,1995,1995,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1995,1995,1995,1995,1995,1995,1995,1995,1995,1995,1995,1994,1994,1994,1994,1994,1994,1994,1994,1994,1994,1994,1993,1993,1993,1993,1993,1993,1993,1993,1993,1992,1992,1992,1992,1992,1992,1992,1992,1992,1991,1991,1991,1991,1991,1991,1991,1991,1990,1990,1990,1990,1990,1990,1990,1990,1989,1989,1989,1989,1989,1989,1989,1988,1988,1988,1988,1988,1988,1988,1987,1987,1987,1987,1987,1987,1987,1986,1986,1986,1986,1986,1986,1985,1985,1985,1985,1985,1985,1984,1984,1984,1984,1984,1984,1983,1983,1983,1983,1983,1983,1982,1982,1982,1982,1982,1982,1981,1981,1981,1981,1981,1980,1980,1980,1980,1980,1980,1979,1979,1979,1979,1979,1978,1978,1978,1978,1978,1977,1977,1977,1977,1977,1976,1976,1976,1976,1976,1975,1975,1975,1975,1974,1974,1974,1974,1974,1973,1973,1973,1973,1973,1972,1972,1972,1972,1971,1971,1971,1971,1970,1970,1970,1970,1970,1969,1969,1969,1969,1968,1968,1968,1968,1967,1967,1967,1967,1966,1966,1966,1966,1965,1965,1965,1965,1964,1964,1964,1964,1963,1963,1963,1963,1962,1962,1962,1962,1961,1961,1961,1961,1960,1960,1960,1959,1959,1959,1959,1958,1958,1958,1958,1957,1957,1957,1956,1956,1956,1956,1955,1955,1955,1954,1954,1954,1954,1953,1953,1953,1952,1952,1952,1952,1951,1951,1951,1950,1950,1950,1949,1949,1949,1949,1948,1948,1948,1947,1947,1947,1946,1946,1946,1945,1945,1945,1945,1944,1944,1944,1943,1943,1943,1942,1942,1942,1941,1941,1941,1940,1940,1940,1939,1939,1939,1938,1938,1938,1937,1937,1937,1936,1936,1936,1935,1935,1935,1934,1934,1934,1933,1933,1933,1932,1932,1932,1931,1931,1931,1930,1930,1929,1929,1929,1928,1928,1928,1927,1927,1927,1926,1926,1925,1925,1925,1924,1924,1924,1923,1923,1923,1922,1922,1921,1921,1921,1920,1920,1920,1919,1919,1918,1918,1918,1917,1917,1916,1916,1916,1915,1915,1915,1914,1914,1913,1913,1913,1912,1912,1911,1911,1911,1910,1910,1909,1909,1909,1908,1908,1907,1907,1907,1906,1906,1905,1905,1905,1904,1904,1903,1903,1902,1902,1902,1901,1901,1900,1900,1900,1899,1899,1898,1898,1897,1897,1897,1896,1896,1895,1895,1894,1894,1894,1893,1893,1892,1892,1891,1891,1891,1890,1890,1889,1889,1888,1888,1887,1887,1887,1886,1886,1885,1885,1884,1884,1883,1883,1882,1882,1882,1881,1881,1880,1880,1879,1879,1878,1878,1877,1877,1876,1876,1876,1875,1875,1874,1874,1873,1873,1872,1872,1871,1871,1870,1870,1869,1869,1868,1868,1868,1867,1867,1866,1866,1865,1866,1866,1867,1867,1868,1868,1868,1869,1869,1870,1870,1871,1871,1872,1872,1873,1873,1874,1874,1875,1875,1876,1876,1876,1877,1877,1878,1878,1879,1879,1880,1880,1881,1881,1882,1882,1882,1883,1883,1884,1884,1885,1885,1886,1886,1887,1887,1887,1888,1888,1889,1889,1890,1890,1891,1891,1891,1892,1892,1893,1893,1894,1894,1894,1895,1895,1896,1896,1897,1897,1897,1898,1898,1899,1899,1900,1900,1900,1901,1901,1902,1902,1902,1903,1903,1904,1904,1905,1905,1905,1906,1906,1907,1907,1907,1908,1908,1909,1909,1909,1910,1910,1911,1911,1911,1912,1912,1913,1913,1913,1914,1914,1915,1915,1915,1916,1916,1916,1917,1917,1918,1918,1918,1919,1919,1920,1920,1920,1921,1921,1921,1922,1922,1923,1923,1923,1924,1924,1924,1925,1925,1925,1926,1926,1927,1927,1927,1928,1928,1928,1929,1929,1929,1930,1930,1931,1931,1931,1932,1932,1932,1933,1933,1933,1934,1934,1934,1935,1935,1935,1936,1936,1936,1937,1937,1937,1938,1938,1938,1939,1939,1939,1940,1940,1940,1941,1941,1941,1942,1942,1942,1943,1943,1943,1944,1944,1944,1945,1945,1945,1945,1946,1946,1946,1947,1947,1947,1948,1948,1948,1949,1949,1949,1949,1950,1950,1950,1951,1951,1951,1952,1952,1952,1952,1953,1953,1953,1954,1954,1954,1954,1955,1955,1955,1956,1956,1956,1956,1957,1957,1957,1958,1958,1958,1958,1959,1959,1959,1959,1960,1960,1960,1961,1961,1961,1961,1962,1962,1962,1962,1963,1963,1963,1963,1964,1964,1964,1964,1965,1965,1965,1965,1966,1966,1966,1966,1967,1967,1967,1967,1968,1968,1968,1968,1969,1969,1969,1969,1970,1970,1970,1970,1970,1971,1971,1971,1971,1972,1972,1972,1972,1973,1973,1973,1973,1973,1974,1974,1974,1974,1974,1975,1975,1975,1975,1976,1976,1976,1976,1976,1977,1977,1977,1977,1977,1978,1978,1978,1978,1978,1979,1979,1979,1979,1979,1980,1980,1980,1980,1980,1980,1981,1981,1981,1981,1981,1982,1982,1982,1982,1982,1982,1983,1983,1983,1983,1983,1983,1984,1984,1984,1984,1984,1984,1985,1985,1985,1985,1985,1985,1986,1986,1986,1986,1986,1986,1987,1987,1987,1987,1987,1987,1987,1988,1988,1988,1988,1988,1988,1988,1989,1989,1989,1989,1989,1989,1989,1990,1990,1990,1990,1990,1990,1990,1990,1991,1991,1991,1991,1991,1991,1991,1991,1992,1992,1992,1992,1992,1992,1992,1992,1992,1993,1993,1993,1993,1993,1993,1993,1993,1993,1994,1994,1994,1994,1994,1994,1994,1994,1994,1994,1994,1995,1995,1995,1995,1995,1995,1995,1995,1995,1995,1995,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1999,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1998,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1997,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1996,1995,1995,1995,1995,1995,1995,1995,1995,1995,1995,1995,1994,1994,1994,1994,1994,1994,1994,1994,1994,1994,1994,1993,1993,1993,1993,1993,1993,1993,1993,1993,1992,1992,1992,1992,1992,1992,1992,1992,1992,1991,1991,1991,1991,1991,1991,1991,1991,1990,1990,1990,1990,1990,1990,1990,1990,1989,1989,1989,1989,1989,1989,1989,1988,1988,1988,1988,1988,1988,1988,1987,1987,1987,1987,1987,1987,1987,1986,1986,1986,1986,1986,1986,1985,1985,1985,1985,1985,1985,1984,1984,1984,1984,1984,1984,1983,1983,1983,1983,1983,1983,1982,1982,1982,1982,1982,1982,1981,1981,1981,1981,1981,1980,1980,1980,1980,1980,1980,1979,1979,1979,1979,1979,1978,1978,1978,1978,1978,1977,1977,1977,1977,1977,1976,1976,1976,1976,1976,1975,1975,1975,1975,1974,1974,1974,1974,1974,1973,1973,1973,1973,1973,1972,1972,1972,1972,1971,1971,1971,1971,1970,1970,1970,1970,1970,1969,1969,1969,1969,1968,1968,1968,1968,1967,1967,1967,1967,1966,1966,1966,1966,1965,1965,1965,1965,1964,1964,1964,1964,1963,1963,1963,1963,1962,1962,1962,1962,1961,1961,1961,1961,1960,1960,1960,1959,1959,1959,1959,1958,1958,1958,1958,1957,1957,1957,1956,1956,1956,1956,1955,1955,1955,1954,1954,1954,1954,1953,1953,1953,1952,1952,1952,1952,1951,1951,1951,1950,1950,1950,1949,1949,1949,1949,1948,1948,1948,1947,1947,1947,1946,1946,1946,1945,1945,1945,1945,1944,1944,1944,1943,1943,1943,1942,1942,1942,1941,1941,1941,1940,1940,1940,1939,1939,1939,1938,1938,1938,1937,1937,1937,1936,1936,1936,1935,1935,1935,1934,1934,1934,1933,1933,1933,1932,1932,1932,1931,1931,1931,1930,1930,1929,1929,1929,1928,1928,1928,1927,1927,1927,1926,1926,1925,1925,1925,1924,1924,1924,1923,1923,1923,1922,1922,1921,1921,1921,1920,1920,1920,1919,1919,1918,1918,1918,1917,1917,1916,1916,1916,1915,1915,1915,1914,1914,1913,1913,1913,1912,1912,1911,1911,1911,1910,1910,1909,1909,1909,1908,1908,1907,1907,1907,1906,1906,1905,1905,1905,1904,1904,1903,1903,1902,1902,1902,1901,1901,1900,1900,1900,1899,1899,1898,1898,1897,1897,1897,1896,1896,1895,1895,1894,1894,1894,1893,1893,1892,1892,1891,1891,1891,1890,1890,1889,1889,1888,1888,1887,1887,1887,1886,1886,1885,1885,1884,1884,1883,1883,1882,1882,1882,1881,1881,1880,1880,1879,1879,1878,1878,1877,1877,1876,1876,1876,1875,1875,1874,1874,1873,1873,1872,1872,1871,1871,1870,1870,1869,1869,1868,1868,1868,1867,1867,1866,1866,1865,1864,1862,1861,1859,1858,1856,1855,1853,1852,1851,1849,1848,1846,1845,1843,1842,1840,1839,1837,1836,1834,1833,1831,1830,1828,1827,1826,1824,1823,1821,1820,1818,1817,1815,1814,1812,1811,1809,1808,1806,1805,1803,1802,1800,1799,1797,1796,1794,1793,1791,1790,1788,1787,1785,1784,1782,1781,1779,1778,1776,1775,1773,1772,1770,1769,1767,1766,1764,1763,1761,1760,1758,1757,1755,1754,1752,1751,1749,1748,1746,1745,1743,1742,1740,1739,1737,1736,1734,1733,1731,1730,1728,1727,1725,1724,1722,1720,1719,1717,1716,1714,1713,1711,1710,1708,1707,1705,1704,1702,1701,1699,1697,1696,1694,1693,1691,1690,1688,1687,1685,1684,1682,1681,1679,1677,1676,1674,1673,1671,1670,1668,1667,1665,1664,1662,1660,1659,1657,1656,1654,1653,1651,1650,1648,1646,1645,1643,1642,1640,1639,1637,1636,1634,1632,1631,1629,1628,1626,1625,1623,1621,1620,1618,1617,1615,1614,1612,1610,1609,1607,1606,1604,1603,1601,1599,1598,1596,1595,1593,1592,1590,1588,1587,1585,1584,1582,1581,1579,1577,1576,1574,1573,1571,1569,1568,1566,1565,1563,1562,1560,1558,1557,1555,1554,1552,1550,1549,1547,1546,1544,1542,1541,1539,1538,1536,1534,1533,1531,1530,1528,1526,1525,1523,1522,1520,1518,1517,1515,1514,1512,1510,1509,1507,1506,1504,1502,1501,1499,1498,1496,1494,1493,1491,1490,1488,1486,1485,1483,1482,1480,1478,1477,1475,1473,1472,1470,1469,1467,1465,1464,1462,1461,1459,1457,1456,1454,1452,1451,1449,1448,1446,1444,1443,1441,1439,1438,1436,1435,1433,1431,1430,1428,1426,1425,1423,1422,1420,1418,1417,1415,1413,1412,1410,1409,1407,1405,1404,1402,1400,1399,1397,1395,1394,1392,1391,1389,1387,1386,1384,1382,1381,1379,1377,1376,1374,1373,1371,1369,1368,1366,1364,1363,1361,1359,1358,1356,1355,1353,1351,1350,1348,1346,1345,1343,1341,1340,1338,1336,1335,1333,1331,1330,1328,1327,1325,1323,1322,1320,1318,1317,1315,1313,1312,1310,1308,1307,1305,1303,1302,1300,1298,1297,1295,1294,1292,1290,1289,1287,1285,1284,1282,1280,1279,1277,1275,1274,1272,1270,1269,1267,1265,1264,1262,1260,1259,1257,1255,1254,1252,1250,1249,1247,1245,1244,1242,1240,1239,1237,1235,1234,1232,1230,1229,1227,1225,1224,1222,1220,1219,1217,1215,1214,1212,1210,1209,1207,1205,1204,1202,1200,1199,1197,1195,1194,1192,1190,1189,1187,1185,1184,1182,1180,1179,1177,1175,1174,1172,1170,1169,1167,1165,1164,1162,1160,1159,1157,1155,1154,1152,1150,1149,1147,1145,1144,1142,1140,1139,1137,1135,1134,1132,1130,1129,1127,1125,1124,1122,1120,1119,1117,1115,1114,1112,1110,1109,1107,1105,1104,1102,1100,1098,1097,1095,1093,1092,1090,1088,1087,1085,1083,1082,1080,1078,1077,1075,1073,1072,1070,1068,1067,1065,1063,1062,1060,1058,1057,1055,1053,1052,1050,1048,1046,1045,1043,1041,1040,1038,1036,1035,1033,1031,1030,1028,1026,1025,1023,1021,1020,1018,1016,1015,1013,1011,1010,1008,1006,1005,1003,1001,999,998,996,994,993,991,989,988,986,984,983,981,979,978,976,974,973,971,969,968,966,964,963,961,959,958,956,954,953,951,949,947,946,944,942,941,939,937,936,934,932,931,929,927,926,924,922,921,919,917,916,914,912,911,909,907,906,904,902,901,899,897,895,894,892,890,889,887,885,884,882,880,879,877,875,874,872,870,869,867,865,864,862,860,859,857,855,854,852,850,849,847,845,844,842,840,839,837,835,834,832,830,829,827,825,824,822,820,819,817,815,814,812,810,809,807,805,804,802,800,799,797,795,794,792,790,789,787,785,784,782,780,779,777,775,774,772,770,769,767,765,764,762,760,759,757,755,754,752,750,749,747,745,744,742,740,739,737,735,734,732,730,729,727,725,724,722,720,719,717,715,714,712,710,709,707,705,704,702,701,699,697,696,694,692,691,689,687,686,684,682,681,679,677,676,674,672,671,669,668,666,664,663,661,659,658,656,654,653,651,649,648,646,644,643,641,640,638,636,635,633,631,630,628,626,625,623,622,620,618,617,615,613,612,610,608,607,605,604,602,600,599,597,595,594,592,590,589,587,586,584,582,581,579,577,576,574,573,571,569,568,566,564,563,561,560,558,556,555,553,551,550,548,547,545,543,542,540,538,537,535,534,532,530,529,527,526,524,522,521,519,517,516,514,513,511,509,508,506,505,503,501,500,498,497,495,493,492,490,489,487,485,484,482,481,479,477,476,474,473,471,469,468,466,465,463,461,460,458,457,455,453,452,450,449,447,445,444,442,441,439,437,436,434,433,431,430,428,426,425,423,422,420,418,417,415,414,412,411,409,407,406,404,403,401,400,398,396,395,393,392,390,389,387,385,384,382,381,379,378,376,374,373,371,370,368,367,365,363,362,360,359,357,356,354,353,351,349,348,346,345,343,342,340,339,337,335,334,332,331,329,328,326,325,323,322,320,318,317,315,314,312,311,309,308,306,305,303,302,300,298,297,295,294,292,291,289,288,286,285,283,282,280,279,277,275,274,272,271,269,268,266,265,263,262,260,259,257,256,254,253,251,250,248,247,245,244,242,241,239,238,236,235,233,232,230,229,227,226,224,223,221,220,218,217,215,214,212,211,209,208,206,205,203,202,200,199,197,196,194,193,191,190,188,187,185,184,182,181,179,178,176,175,173,172,171,169,168,166,165,163,162,160,159,157,156,154,153,151,150,148,147,146,144,143,141,140,138,137,135,134,133,133,132,132,131,131,131,130,130,129,129,128,128,127,127,126,126,125,125,124,124,123,123,123,122,122,121,121,120,120,119,119,118,118,117,117,117,116,116,115,115,114,114,113,113,112,112,112,111,111,110,110,109,109,108,108,108,107,107,106,106,105,105,105,104,104,103,103,102,102,102,101,101,100,100,99,99,99,98,98,97,97,97,96,96,95,95,94,94,94,93,93,92,92,92,91,91,90,90,90,89,89,88,88,88,87,87,86,86,86,85,85,84,84,84,83,83,83,82,82,81,81,81,80,80,79,79,79,78,78,78,77,77,76,76,76,75,75,75,74,74,74,73,73,72,72,72,71,71,71,70,70,70,69,69,68,68,68,67,67,67,66,66,66,65,65,65,64,64,64,63,63,63,62,62,62,61,61,61,60,60,60,59,59,59,58,58,58,57,57,57,56,56,56,55,55,55,54,54,54,54,53,53,53,52,52,52,51,51,51,50,50,50,50,49,49,49,48,48,48,47,47,47,47,46,46,46,45,45,45,45,44,44,44,43,43,43,43,42,42,42,41,41,41,41,40,40,40,40,39,39,39,38,38,38,38,37,37,37,37,36,36,36,36,35,35,35,35,34,34,34,34,33,33,33,33,32,32,32,32,31,31,31,31,30,30,30,30,29,29,29,29,29,28,28,28,28,27,27,27,27,26,26,26,26,26,25,25,25,25,25,24,24,24,24,23,23,23,23,23,22,22,22,22,22,21,21,21,21,21,20,20,20,20,20,19,19,19,19,19,19,18,18,18,18,18,17,17,17,17,17,17,16,16,16,16,16,16,15,15,15,15,15,15,14,14,14,14,14,14,13,13,13,13,13,13,12,12,12,12,12,12,12,11,11,11,11,11,11,11,10,10,10,10,10,10,10,9,9,9,9,9,9,9,9,8,8,8,8,8,8,8,8,7,7,7,7,7,7,7,7,7,6,6,6,6,6,6,6,6,6,5,5,5,5,5,5,5,5,5,5,5,4,4,4,4,4,4,4,4,4,4,4,3,3,3,3,3,3,3,3,3,3,3,3,3,3,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4,4,4,4,5,5,5,5,5,5,5,5,5,5,5,6,6,6,6,6,6,6,6,6,7,7,7,7,7,7,7,7,7,8,8,8,8,8,8,8,8,9,9,9,9,9,9,9,9,10,10,10,10,10,10,10,11,11,11,11,11,11,11,12,12,12,12,12,12,12,13,13,13,13,13,13,14,14,14,14,14,14,15,15,15,15,15,15,16,16,16,16,16,16,17,17,17,17,17,17,18,18,18,18,18,19,19,19,19,19,19,20,20,20,20,20,21,21,21,21,21,22,22,22,22,22,23,23,23,23,23,24,24,24,24,25,25,25,25,25,26,26,26,26,26,27,27,27,27,28,28,28,28,29,29,29,29,29,30,30,30,30,31,31,31,31,32,32,32,32,33,33,33,33,34,34,34,34,35,35,35,35,36,36,36,36,37,37,37,37,38,38,38,38,39,39,39,40,40,40,40,41,41,41,41,42,42,42,43,43,43,43,44,44,44,45,45,45,45,46,46,46,47,47,47,47,48,48,48,49,49,49,50,50,50,50,51,51,51,52,52,52,53,53,53,54,54,54,54,55,55,55,56,56,56,57,57,57,58,58,58,59,59,59,60,60,60,61,61,61,62,62,62,63,63,63,64,64,64,65,65,65,66,66,66,67,67,67,68,68,68,69,69,70,70,70,71,71,71,72,72,72,73,73,74,74,74,75,75,75,76,76,76,77,77,78,78,78,79,79,79,80,80,81,81,81,82,82,83,83,83,84,84,84,85,85,86,86,86,87,87,88,88,88,89,89,90,90,90,91,91,92,92,92,93,93,94,94,94,95,95,96,96,97,97,97,98,98,99,99,99,100,100,101,101,102,102,102,103,103,104,104,105,105,105,106,106,107,107,108,108,108,109,109,110,110,111,111,112,112,112,113,113,114,114,115,115,116,116,117,117,117,118,118,119,119,120,120,121,121,122,122,123,123,123,124,124,125,125,126,126,127,127,128,128,129,129,130,130,131,131,131,132,132,133,133};
volatile int32_t s32_svpwm_inc_0, s32_svpwm_idx_A0, s32_svpwm_idx_B0, s32_svpwm_idx_C0;
volatile int32_t s32_svpwm_inc_1, s32_svpwm_idx_A1, s32_svpwm_idx_B1, s32_svpwm_idx_C1;
volatile uint32_t u32_ts_motor_0;
static uint32_t u32_ts_motor_1;
uint32_t u32_counter_timer = 0;

/* Private function prototypes -----------------------------------------------*/
static void v_Board_Init(void);
static void v_BLDC_Ctl_Init(void);
static void v_Set_BLDC_Speed(ENUM_MOTOR_T enum_motor, int32_t s32_speed);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{
  v_Board_Init();
  
  /* Infinite loop */
  while (1)
  {
    v_BLDC_Receive();

    if (u32_tick_cnt_0 >= u32_ts_motor_0) // unit: 10us
    {
      u32_tick_cnt_0 = 0;
      if (s32_svpwm_inc_0 != 0) // if need shift voltage
      {
        /* Increase index A0 B0 C0 with s32_svpwm_inc_0 */
        s32_svpwm_idx_A0 += s32_svpwm_inc_0;
        if (s32_svpwm_idx_A0 >= SVPWM_TABLE_SIZE) s32_svpwm_idx_A0 = 0;
        else if (s32_svpwm_idx_A0 < 0) s32_svpwm_idx_A0 = SVPWM_TABLE_SIZE - 1;
        
        s32_svpwm_idx_B0 += s32_svpwm_inc_0;
        if (s32_svpwm_idx_B0 >= SVPWM_TABLE_SIZE) s32_svpwm_idx_B0 = 0;
        else if (s32_svpwm_idx_B0 < 0) s32_svpwm_idx_B0 = SVPWM_TABLE_SIZE - 1;
        
        s32_svpwm_idx_C0 += s32_svpwm_inc_0;
        if (s32_svpwm_idx_C0 >= SVPWM_TABLE_SIZE) s32_svpwm_idx_C0 = 0;
        else if (s32_svpwm_idx_C0 < 0) s32_svpwm_idx_C0 = SVPWM_TABLE_SIZE - 1;
        
        /* Write desired pwm */
        v_PWM_Set(MOTOR_0, u16_svpwm_table[s32_svpwm_idx_A0], 
                  u16_svpwm_table[s32_svpwm_idx_B0], u16_svpwm_table[s32_svpwm_idx_C0]);
      }
    }
    
    if (u32_tick_cnt_1 >= u32_ts_motor_1) // unit: 10us
    {
      u32_tick_cnt_1 = 0;
      if (s32_svpwm_inc_1 != 0) // if need shift voltage
      {
        /* Increase index A1 B1 C1 with s32_svpwm_inc_1 */
        s32_svpwm_idx_A1 += s32_svpwm_inc_1;
        if (s32_svpwm_idx_A1 >= SVPWM_TABLE_SIZE) s32_svpwm_idx_A1 = 0;
        else if (s32_svpwm_idx_A1 < 0) s32_svpwm_idx_A1 = SVPWM_TABLE_SIZE - 1;
        
        s32_svpwm_idx_B1 += s32_svpwm_inc_1;
        if (s32_svpwm_idx_B1 >= SVPWM_TABLE_SIZE) s32_svpwm_idx_B1 = 0;
        else if (s32_svpwm_idx_B1 < 0) s32_svpwm_idx_B1 = SVPWM_TABLE_SIZE - 1;
        
        s32_svpwm_idx_C1 += s32_svpwm_inc_1;
        if (s32_svpwm_idx_C1 >= SVPWM_TABLE_SIZE) s32_svpwm_idx_C1 = 0;
        else if (s32_svpwm_idx_C1 < 0) s32_svpwm_idx_C1 = SVPWM_TABLE_SIZE - 1;
        
        /* Write desired pwm */
        v_PWM_Set(MOTOR_1, u16_svpwm_table[s32_svpwm_idx_A1], 
                  u16_svpwm_table[s32_svpwm_idx_B1], u16_svpwm_table[s32_svpwm_idx_C1]);
      }
    }
    
  }
}

/**
  * @brief  Init board
  * @param  None
  * @retval None
  */
static void v_Board_Init(void)
{
  /* Check System Clock*/
  RCC_ClocksTypeDef RCC_ClocksStructure;
  uint8_t u8_clock_source;
  u8_clock_source = RCC_GetSYSCLKSource();
  if (u8_clock_source != 0x08) // 0x08: PLL used as system clock
  {
    //while (true);
  }
  RCC_GetClocksFreq(&RCC_ClocksStructure);
  if (RCC_ClocksStructure.SYSCLK_Frequency != 72000000)
  {
    //while (true);
  }
  
  /* Enable SysTick at 1ms interrupt */
  //SysTick_Config(SystemCoreClock / F_CTRL);
  
  /* Driver Initialization */
  v_GPIO_Init();
  //v_Green_On();
  
  v_Motor_Init();
  //v_PWM_Set(MOTOR_0, 0, 900, 1799);
  
  v_UART_Comm_Init();
  
  v_BLDC_Ctl_Init();
}

/**
  * @brief  BLDC Init
  * @param  None
  * @retval None
  */
static void v_BLDC_Ctl_Init(void)
{
  /* MOTOR 0 */
  s32_svpwm_inc_0 = 0;
  s32_svpwm_idx_A0 = 0;
  s32_svpwm_idx_B0 = s32_svpwm_idx_A0 + SVPWM_TABLE_SIZE/3;
  s32_svpwm_idx_C0 = s32_svpwm_idx_B0 + SVPWM_TABLE_SIZE/3;
  v_PWM_Set(MOTOR_0, 0, 0, 0);
  v_Set_BLDC_Speed(MOTOR_0, 0);
  
  /* MOTOR 1 */
  s32_svpwm_inc_1 = 0;
  s32_svpwm_idx_A1 = 0;
  s32_svpwm_idx_B1 = s32_svpwm_idx_A1 + SVPWM_TABLE_SIZE/3;
  s32_svpwm_idx_C1 = s32_svpwm_idx_B1 + SVPWM_TABLE_SIZE/3;
  v_PWM_Set(MOTOR_1, 0, 0, 0);
  v_Set_BLDC_Speed(MOTOR_1, 0);
}

/**
  * @brief  Set BLDC Speed (Hz) (x1000)
  * @param  ...
  * @param  ...
  * @retval None
  */
static void v_Set_BLDC_Speed(ENUM_MOTOR_T enum_motor, int32_t s32_speed)
{
  float flt_ts_motor;
  
  switch (enum_motor)
  {
    case MOTOR_0: 
      if (s32_speed == 0)
      {
        s32_svpwm_inc_0  = 0;
      }
      else
      {
        if (s32_speed > 0) s32_svpwm_inc_0 = 1;
        else
        {
          s32_speed = -s32_speed;
          s32_svpwm_inc_0 = -1;
        }
        
        /* Calculate ts in 10us */
        flt_ts_motor = SPEED_CONVERT / (float)s32_speed;
        u32_ts_motor_0 = (uint32_t)flt_ts_motor;
      }
      break;
    case MOTOR_1:
      if (s32_speed == 0)
      {
        s32_svpwm_inc_1  = 0;
      }
      else
      {
        if (s32_speed > 0) s32_svpwm_inc_1 = 1;
        else
        {
          s32_speed = -s32_speed;
          s32_svpwm_inc_1 = -1;
        }
        
        /* Calculate ts in 10us */
        flt_ts_motor = SPEED_CONVERT / (float)s32_speed;
        u32_ts_motor_1 = (uint32_t)flt_ts_motor;
      }
      break;
    default:
      break;
  }
}

bool bool_Set_BLDC_Speed_Handler(uint8_t u8_msg_id, uint8_t *pu8_payload, uint32_t u32_payload_cnt)
{
  int16_t s16_desired_speed;
  
  if (*pu8_payload == 0x03)
  {
    s16_desired_speed = (*(pu8_payload + 1) << 8) & 0x0ff00;
    s16_desired_speed += *(pu8_payload + 2) & 0x0ff;
    v_Set_BLDC_Speed(MOTOR_0, (int32_t)s16_desired_speed);
    
    s16_desired_speed = (*(pu8_payload + 3) << 8) & 0x0ff00;
    s16_desired_speed += *(pu8_payload + 4) & 0x0ff;
    v_Set_BLDC_Speed(MOTOR_1, (int32_t)s16_desired_speed);
  }
  else if (*pu8_payload == 0x01)
  {
    s16_desired_speed = (*(pu8_payload + 1) << 8) & 0x0ff00;
    s16_desired_speed += *(pu8_payload + 2) & 0x0ff;
    v_Set_BLDC_Speed(MOTOR_0, (int32_t)s16_desired_speed);
  }
  else if (*pu8_payload == 0x02)
  {
    s16_desired_speed = (*(pu8_payload + 3) << 8) & 0x0ff00;
    s16_desired_speed += *(pu8_payload + 4) & 0x0ff;
    v_Set_BLDC_Speed(MOTOR_1, (int32_t)s16_desired_speed);
  }
  
//  au8_respond_payload[0] = *pu8_payload;
//  au8_respond_payload[1] = 0x00; //Ok
//  v_Send_Response(u8_msg_id, au8_respond_payload, 2);
  v_Red_Toggle();
  return true;
}

void MOTOR_TIMER_IRQ_Handler(void)
{
  TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
  
  u32_counter_timer++;
  if (u32_counter_timer == 1)
  {
    u32_counter_timer = 0;
    v_Red_Toggle();
    
    if (s32_svpwm_inc_0 != 0) // if need shift voltage
    {
      /* Increase index A0 B0 C0 with s32_svpwm_inc_0 */
      s32_svpwm_idx_A0 += s32_svpwm_inc_0;
      if (s32_svpwm_idx_A0 >= SVPWM_TABLE_SIZE) s32_svpwm_idx_A0 = 0;
      else if (s32_svpwm_idx_A0 < 0) s32_svpwm_idx_A0 = SVPWM_TABLE_SIZE - 1;
      
      s32_svpwm_idx_B0 += s32_svpwm_inc_0;
      if (s32_svpwm_idx_B0 >= SVPWM_TABLE_SIZE) s32_svpwm_idx_B0 = 0;
      else if (s32_svpwm_idx_B0 < 0) s32_svpwm_idx_B0 = SVPWM_TABLE_SIZE - 1;
      
      s32_svpwm_idx_C0 += s32_svpwm_inc_0;
      if (s32_svpwm_idx_C0 >= SVPWM_TABLE_SIZE) s32_svpwm_idx_C0 = 0;
      else if (s32_svpwm_idx_C0 < 0) s32_svpwm_idx_C0 = SVPWM_TABLE_SIZE - 1;
      
      /* Write desired pwm */
      v_PWM_Set(MOTOR_0, u16_svpwm_table[s32_svpwm_idx_A0], 
                u16_svpwm_table[s32_svpwm_idx_B0], u16_svpwm_table[s32_svpwm_idx_C0]);
    }
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
    
  }
}
#endif

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
