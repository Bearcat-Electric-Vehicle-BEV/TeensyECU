/** 
 * Author: Marshal Stewart
 * Date: 03-Jul-2022 23:24:59
 *
 * The following is an auto-generated file edits are likely to be
 * overwritten.
 */

#ifndef ETC_CURVE_H
#define ETC_CURVE_H

#include "bev_etc.h"

#define TABLE_SIZE 2301U

/**
 * @paragraph ETC Lookup Table
 * The following array is the lookup table used for ETC calculations.
 * A lookup table is used instead of a calculations for speed. Flash 
 * memory is barely being used, therefore placing this table in flash
 * saves Teensy from doing complicated polynomial floating point calcs
 * every 50 ms or so.
 *
 * Takes input in integer Nm * 10, outputs integer in Nm * 10
 *
 */
static const torque_t lookup[TABLE_SIZE] = {

0x0, 0xA, 0x14, 0x1E, 0x28, 
0x32, 0x3C, 0x46, 0x50, 0x5A, 
0x64, 0x6E, 0x78, 0x82, 0x8C, 
0x96, 0xA0, 0xAA, 0xB4, 0xBE, 
0xC8, 0xD2, 0xDC, 0xE6, 0xF0, 
0xFA, 0x104, 0x10E, 0x118, 0x122, 
0x12C, 0x136, 0x140, 0x14A, 0x154, 
0x15E, 0x168, 0x172, 0x17C, 0x186, 
0x190, 0x19A, 0x1A4, 0x1AE, 0x1B8, 
0x1C2, 0x1CC, 0x1D6, 0x1E0, 0x1EA, 
0x1F4, 0x1FE, 0x208, 0x212, 0x21C, 
0x226, 0x230, 0x23A, 0x244, 0x24E, 
0x258, 0x262, 0x26C, 0x276, 0x280, 
0x28A, 0x294, 0x29E, 0x2A8, 0x2B2, 
0x2BC, 0x2C6, 0x2D0, 0x2DA, 0x2E4, 
0x2EE, 0x2F8, 0x302, 0x30C, 0x316, 
0x320, 0x32A, 0x334, 0x33E, 0x348, 
0x352, 0x35C, 0x366, 0x370, 0x37A, 
0x384, 0x38E, 0x398, 0x3A2, 0x3AC, 
0x3B6, 0x3C0, 0x3CA, 0x3D4, 0x3DE, 
0x3E8, 0x3F2, 0x3FC, 0x406, 0x410, 
0x41A, 0x424, 0x42E, 0x438, 0x442, 
0x44C, 0x456, 0x460, 0x46A, 0x474, 
0x47E, 0x488, 0x492, 0x49C, 0x4A6, 
0x4B0, 0x4BA, 0x4C4, 0x4CE, 0x4D8, 
0x4E2, 0x4EC, 0x4F6, 0x500, 0x50A, 
0x514, 0x51E, 0x528, 0x532, 0x53C, 
0x546, 0x550, 0x55A, 0x564, 0x56E, 
0x578, 0x582, 0x58C, 0x596, 0x5A0, 
0x5AA, 0x5B4, 0x5BE, 0x5C8, 0x5D2, 
0x5DC, 0x5E6, 0x5F0, 0x5FA, 0x604, 
0x60E, 0x618, 0x622, 0x62C, 0x636, 
0x640, 0x64A, 0x654, 0x65E, 0x668, 
0x672, 0x67C, 0x686, 0x690, 0x69A, 
0x6A4, 0x6AE, 0x6B8, 0x6C2, 0x6CC, 
0x6D6, 0x6E0, 0x6EA, 0x6F4, 0x6FE, 
0x708, 0x712, 0x71C, 0x726, 0x730, 
0x73A, 0x744, 0x74E, 0x758, 0x762, 
0x76C, 0x776, 0x780, 0x78A, 0x794, 
0x79E, 0x7A8, 0x7B2, 0x7BC, 0x7C6, 
0x7D0, 0x7DA, 0x7E4, 0x7EE, 0x7F8, 
0x802, 0x80C, 0x816, 0x820, 0x82A, 
0x834, 0x83E, 0x848, 0x852, 0x85C, 
0x866, 0x870, 0x87A, 0x884, 0x88E, 
0x898, 0x8A2, 0x8AC, 0x8B6, 0x8C0, 
0x8CA, 0x8D4, 0x8DE, 0x8E8, 0x8F2, 
0x8FC, 0x906, 0x910, 0x91A, 0x924, 
0x92E, 0x938, 0x942, 0x94C, 0x956, 
0x960, 0x96A, 0x974, 0x97E, 0x988, 
0x992, 0x99C, 0x9A6, 0x9B0, 0x9BA, 
0x9C4, 0x9CE, 0x9D8, 0x9E2, 0x9EC, 
0x9F6, 0xA00, 0xA0A, 0xA14, 0xA1E, 
0xA28, 0xA32, 0xA3C, 0xA46, 0xA50, 
0xA5A, 0xA64, 0xA6E, 0xA78, 0xA82, 
0xA8C, 0xA96, 0xAA0, 0xAAA, 0xAB4, 
0xABE, 0xAC8, 0xAD2, 0xADC, 0xAE6, 
0xAF0, 0xAFA, 0xB04, 0xB0E, 0xB18, 
0xB22, 0xB2C, 0xB36, 0xB40, 0xB4A, 
0xB54, 0xB5E, 0xB68, 0xB72, 0xB7C, 
0xB86, 0xB90, 0xB9A, 0xBA4, 0xBAE, 
0xBB8, 0xBC2, 0xBCC, 0xBD6, 0xBE0, 
0xBEA, 0xBF4, 0xBFE, 0xC08, 0xC12, 
0xC1C, 0xC26, 0xC30, 0xC3A, 0xC44, 
0xC4E, 0xC58, 0xC62, 0xC6C, 0xC76, 
0xC80, 0xC8A, 0xC94, 0xC9E, 0xCA8, 
0xCB2, 0xCBC, 0xCC6, 0xCD0, 0xCDA, 
0xCE4, 0xCEE, 0xCF8, 0xD02, 0xD0C, 
0xD16, 0xD20, 0xD2A, 0xD34, 0xD3E, 
0xD48, 0xD52, 0xD5C, 0xD66, 0xD70, 
0xD7A, 0xD84, 0xD8E, 0xD98, 0xDA2, 
0xDAC, 0xDB6, 0xDC0, 0xDCA, 0xDD4, 
0xDDE, 0xDE8, 0xDF2, 0xDFC, 0xE06, 
0xE10, 0xE1A, 0xE24, 0xE2E, 0xE38, 
0xE42, 0xE4C, 0xE56, 0xE60, 0xE6A, 
0xE74, 0xE7E, 0xE88, 0xE92, 0xE9C, 
0xEA6, 0xEB0, 0xEBA, 0xEC4, 0xECE, 
0xED8, 0xEE2, 0xEEC, 0xEF6, 0xF00, 
0xF0A, 0xF14, 0xF1E, 0xF28, 0xF32, 
0xF3C, 0xF46, 0xF50, 0xF5A, 0xF64, 
0xF6E, 0xF78, 0xF82, 0xF8C, 0xF96, 
0xFA0, 0xFAA, 0xFB4, 0xFBE, 0xFC8, 
0xFD2, 0xFDC, 0xFE6, 0xFF0, 0xFFA, 
0x1004, 0x100E, 0x1018, 0x1022, 0x102C, 
0x1036, 0x1040, 0x104A, 0x1054, 0x105E, 
0x1068, 0x1072, 0x107C, 0x1086, 0x1090, 
0x109A, 0x10A4, 0x10AE, 0x10B8, 0x10C2, 
0x10CC, 0x10D6, 0x10E0, 0x10EA, 0x10F4, 
0x10FE, 0x1108, 0x1112, 0x111C, 0x1126, 
0x1130, 0x113A, 0x1144, 0x114E, 0x1158, 
0x1162, 0x116C, 0x1176, 0x1180, 0x118A, 
0x1194, 0x119E, 0x11A8, 0x11B2, 0x11BC, 
0x11C6, 0x11D0, 0x11DA, 0x11E4, 0x11EE, 
0x11F8, 0x1202, 0x120C, 0x1216, 0x1220, 
0x122A, 0x1234, 0x123E, 0x1248, 0x1252, 
0x125C, 0x1266, 0x1270, 0x127A, 0x1284, 
0x128E, 0x1298, 0x12A2, 0x12AC, 0x12B6, 
0x12C0, 0x12CA, 0x12D4, 0x12DE, 0x12E8, 
0x12F2, 0x12FC, 0x1306, 0x1310, 0x131A, 
0x1324, 0x132E, 0x1338, 0x1342, 0x134C, 
0x1356, 0x1360, 0x136A, 0x1374, 0x137E, 
0x1388, 0x1392, 0x139C, 0x13A6, 0x13B0, 
0x13BA, 0x13C4, 0x13CE, 0x13D8, 0x13E2, 
0x13EC, 0x13F6, 0x1400, 0x140A, 0x1414, 
0x141E, 0x1428, 0x1432, 0x143C, 0x1446, 
0x1450, 0x145A, 0x1464, 0x146E, 0x1478, 
0x1482, 0x148C, 0x1496, 0x14A0, 0x14AA, 
0x14B4, 0x14BE, 0x14C8, 0x14D2, 0x14DC, 
0x14E6, 0x14F0, 0x14FA, 0x1504, 0x150E, 
0x1518, 0x1522, 0x152C, 0x1536, 0x1540, 
0x154A, 0x1554, 0x155E, 0x1568, 0x1572, 
0x157C, 0x1586, 0x1590, 0x159A, 0x15A4, 
0x15AE, 0x15B8, 0x15C2, 0x15CC, 0x15D6, 
0x15E0, 0x15EA, 0x15F4, 0x15FE, 0x1608, 
0x1612, 0x161C, 0x1626, 0x1630, 0x163A, 
0x1644, 0x164E, 0x1658, 0x1662, 0x166C, 
0x1676, 0x1680, 0x168A, 0x1694, 0x169E, 
0x16A8, 0x16B2, 0x16BC, 0x16C6, 0x16D0, 
0x16DA, 0x16E4, 0x16EE, 0x16F8, 0x1702, 
0x170C, 0x1716, 0x1720, 0x172A, 0x1734, 
0x173E, 0x1748, 0x1752, 0x175C, 0x1766, 
0x1770, 0x177A, 0x1784, 0x178E, 0x1798, 
0x17A2, 0x17AC, 0x17B6, 0x17C0, 0x17CA, 
0x17D4, 0x17DE, 0x17E8, 0x17F2, 0x17FC, 
0x1806, 0x1810, 0x181A, 0x1824, 0x182E, 
0x1838, 0x1842, 0x184C, 0x1856, 0x1860, 
0x186A, 0x1874, 0x187E, 0x1888, 0x1892, 
0x189C, 0x18A6, 0x18B0, 0x18BA, 0x18C4, 
0x18CE, 0x18D8, 0x18E2, 0x18EC, 0x18F6, 
0x1900, 0x190A, 0x1914, 0x191E, 0x1928, 
0x1932, 0x193C, 0x1946, 0x1950, 0x195A, 
0x1964, 0x196E, 0x1978, 0x1982, 0x198C, 
0x1996, 0x19A0, 0x19AA, 0x19B4, 0x19BE, 
0x19C8, 0x19D2, 0x19DC, 0x19E6, 0x19F0, 
0x19FA, 0x1A04, 0x1A0E, 0x1A18, 0x1A22, 
0x1A2C, 0x1A36, 0x1A40, 0x1A4A, 0x1A54, 
0x1A5E, 0x1A68, 0x1A72, 0x1A7C, 0x1A86, 
0x1A90, 0x1A9A, 0x1AA4, 0x1AAE, 0x1AB8, 
0x1AC2, 0x1ACC, 0x1AD6, 0x1AE0, 0x1AEA, 
0x1AF4, 0x1AFE, 0x1B08, 0x1B12, 0x1B1C, 
0x1B26, 0x1B30, 0x1B3A, 0x1B44, 0x1B4E, 
0x1B58, 0x1B62, 0x1B6C, 0x1B76, 0x1B80, 
0x1B8A, 0x1B94, 0x1B9E, 0x1BA8, 0x1BB2, 
0x1BBC, 0x1BC6, 0x1BD0, 0x1BDA, 0x1BE4, 
0x1BEE, 0x1BF8, 0x1C02, 0x1C0C, 0x1C16, 
0x1C20, 0x1C2A, 0x1C34, 0x1C3E, 0x1C48, 
0x1C52, 0x1C5C, 0x1C66, 0x1C70, 0x1C7A, 
0x1C84, 0x1C8E, 0x1C98, 0x1CA2, 0x1CAC, 
0x1CB6, 0x1CC0, 0x1CCA, 0x1CD4, 0x1CDE, 
0x1CE8, 0x1CF2, 0x1CFC, 0x1D06, 0x1D10, 
0x1D1A, 0x1D24, 0x1D2E, 0x1D38, 0x1D42, 
0x1D4C, 0x1D56, 0x1D60, 0x1D6A, 0x1D74, 
0x1D7E, 0x1D88, 0x1D92, 0x1D9C, 0x1DA6, 
0x1DB0, 0x1DBA, 0x1DC4, 0x1DCE, 0x1DD8, 
0x1DE2, 0x1DEC, 0x1DF6, 0x1E00, 0x1E0A, 
0x1E14, 0x1E1E, 0x1E28, 0x1E32, 0x1E3C, 
0x1E46, 0x1E50, 0x1E5A, 0x1E64, 0x1E6E, 
0x1E78, 0x1E82, 0x1E8C, 0x1E96, 0x1EA0, 
0x1EAA, 0x1EB4, 0x1EBE, 0x1EC8, 0x1ED2, 
0x1EDC, 0x1EE6, 0x1EF0, 0x1EFA, 0x1F04, 
0x1F0E, 0x1F18, 0x1F22, 0x1F2C, 0x1F36, 
0x1F40, 0x1F4A, 0x1F54, 0x1F5E, 0x1F68, 
0x1F72, 0x1F7C, 0x1F86, 0x1F90, 0x1F9A, 
0x1FA4, 0x1FAE, 0x1FB8, 0x1FC2, 0x1FCC, 
0x1FD6, 0x1FE0, 0x1FEA, 0x1FF4, 0x1FFE, 
0x2008, 0x2012, 0x201C, 0x2026, 0x2030, 
0x203A, 0x2044, 0x204E, 0x2058, 0x2062, 
0x206C, 0x2076, 0x2080, 0x208A, 0x2094, 
0x209E, 0x20A8, 0x20B2, 0x20BC, 0x20C6, 
0x20D0, 0x20DA, 0x20E4, 0x20EE, 0x20F8, 
0x2102, 0x210C, 0x2116, 0x2120, 0x212A, 
0x2134, 0x213E, 0x2148, 0x2152, 0x215C, 
0x2166, 0x2170, 0x217A, 0x2184, 0x218E, 
0x2198, 0x21A2, 0x21AC, 0x21B6, 0x21C0, 
0x21CA, 0x21D4, 0x21DE, 0x21E8, 0x21F2, 
0x21FC, 0x2206, 0x2210, 0x221A, 0x2224, 
0x222E, 0x2238, 0x2242, 0x224C, 0x2256, 
0x2260, 0x226A, 0x2274, 0x227E, 0x2288, 
0x2292, 0x229C, 0x22A6, 0x22B0, 0x22BA, 
0x22C4, 0x22CE, 0x22D8, 0x22E2, 0x22EC, 
0x22F6, 0x2300, 0x230A, 0x2314, 0x231E, 
0x2328, 0x2332, 0x233C, 0x2346, 0x2350, 
0x235A, 0x2364, 0x236E, 0x2378, 0x2382, 
0x238C, 0x2396, 0x23A0, 0x23AA, 0x23B4, 
0x23BE, 0x23C8, 0x23D2, 0x23DC, 0x23E6, 
0x23F0, 0x23FA, 0x2404, 0x240E, 0x2418, 
0x2422, 0x242C, 0x2436, 0x2440, 0x244A, 
0x2454, 0x245E, 0x2468, 0x2472, 0x247C, 
0x2486, 0x2490, 0x249A, 0x24A4, 0x24AE, 
0x24B8, 0x24C2, 0x24CC, 0x24D6, 0x24E0, 
0x24EA, 0x24F4, 0x24FE, 0x2508, 0x2512, 
0x251C, 0x2526, 0x2530, 0x253A, 0x2544, 
0x254E, 0x2558, 0x2562, 0x256C, 0x2576, 
0x2580, 0x258A, 0x2594, 0x259E, 0x25A8, 
0x25B2, 0x25BC, 0x25C6, 0x25D0, 0x25DA, 
0x25E4, 0x25EE, 0x25F8, 0x2602, 0x260C, 
0x2616, 0x2620, 0x262A, 0x2634, 0x263E, 
0x2648, 0x2652, 0x265C, 0x2666, 0x2670, 
0x267A, 0x2684, 0x268E, 0x2698, 0x26A2, 
0x26AC, 0x26B6, 0x26C0, 0x26CA, 0x26D4, 
0x26DE, 0x26E8, 0x26F2, 0x26FC, 0x2706, 
0x2710, 0x271A, 0x2724, 0x272E, 0x2738, 
0x2742, 0x274C, 0x2756, 0x2760, 0x276A, 
0x2774, 0x277E, 0x2788, 0x2792, 0x279C, 
0x27A6, 0x27B0, 0x27BA, 0x27C4, 0x27CE, 
0x27D8, 0x27E2, 0x27EC, 0x27F6, 0x2800, 
0x280A, 0x2814, 0x281E, 0x2828, 0x2832, 
0x283C, 0x2846, 0x2850, 0x285A, 0x2864, 
0x286E, 0x2878, 0x2882, 0x288C, 0x2896, 
0x28A0, 0x28AA, 0x28B4, 0x28BE, 0x28C8, 
0x28D2, 0x28DC, 0x28E6, 0x28F0, 0x28FA, 
0x2904, 0x290E, 0x2918, 0x2922, 0x292C, 
0x2936, 0x2940, 0x294A, 0x2954, 0x295E, 
0x2968, 0x2972, 0x297C, 0x2986, 0x2990, 
0x299A, 0x29A4, 0x29AE, 0x29B8, 0x29C2, 
0x29CC, 0x29D6, 0x29E0, 0x29EA, 0x29F4, 
0x29FE, 0x2A08, 0x2A12, 0x2A1C, 0x2A26, 
0x2A30, 0x2A3A, 0x2A44, 0x2A4E, 0x2A58, 
0x2A62, 0x2A6C, 0x2A76, 0x2A80, 0x2A8A, 
0x2A94, 0x2A9E, 0x2AA8, 0x2AB2, 0x2ABC, 
0x2AC6, 0x2AD0, 0x2ADA, 0x2AE4, 0x2AEE, 
0x2AF8, 0x2B02, 0x2B0C, 0x2B16, 0x2B20, 
0x2B2A, 0x2B34, 0x2B3E, 0x2B48, 0x2B52, 
0x2B5C, 0x2B66, 0x2B70, 0x2B7A, 0x2B84, 
0x2B8E, 0x2B98, 0x2BA2, 0x2BAC, 0x2BB6, 
0x2BC0, 0x2BCA, 0x2BD4, 0x2BDE, 0x2BE8, 
0x2BF2, 0x2BFC, 0x2C06, 0x2C10, 0x2C1A, 
0x2C24, 0x2C2E, 0x2C38, 0x2C42, 0x2C4C, 
0x2C56, 0x2C60, 0x2C6A, 0x2C74, 0x2C7E, 
0x2C88, 0x2C92, 0x2C9C, 0x2CA6, 0x2CB0, 
0x2CBA, 0x2CC4, 0x2CCE, 0x2CD8, 0x2CE2, 
0x2CEC, 0x2CF6, 0x2D00, 0x2D0A, 0x2D14, 
0x2D1E, 0x2D28, 0x2D32, 0x2D3C, 0x2D46, 
0x2D50, 0x2D5A, 0x2D64, 0x2D6E, 0x2D78, 
0x2D82, 0x2D8C, 0x2D96, 0x2DA0, 0x2DAA, 
0x2DB4, 0x2DBE, 0x2DC8, 0x2DD2, 0x2DDC, 
0x2DE6, 0x2DF0, 0x2DFA, 0x2E04, 0x2E0E, 
0x2E18, 0x2E22, 0x2E2C, 0x2E36, 0x2E40, 
0x2E4A, 0x2E54, 0x2E5E, 0x2E68, 0x2E72, 
0x2E7C, 0x2E86, 0x2E90, 0x2E9A, 0x2EA4, 
0x2EAE, 0x2EB8, 0x2EC2, 0x2ECC, 0x2ED6, 
0x2EE0, 0x2EEA, 0x2EF4, 0x2EFE, 0x2F08, 
0x2F12, 0x2F1C, 0x2F26, 0x2F30, 0x2F3A, 
0x2F44, 0x2F4E, 0x2F58, 0x2F62, 0x2F6C, 
0x2F76, 0x2F80, 0x2F8A, 0x2F94, 0x2F9E, 
0x2FA8, 0x2FB2, 0x2FBC, 0x2FC6, 0x2FD0, 
0x2FDA, 0x2FE4, 0x2FEE, 0x2FF8, 0x3002, 
0x300C, 0x3016, 0x3020, 0x302A, 0x3034, 
0x303E, 0x3048, 0x3052, 0x305C, 0x3066, 
0x3070, 0x307A, 0x3084, 0x308E, 0x3098, 
0x30A2, 0x30AC, 0x30B6, 0x30C0, 0x30CA, 
0x30D4, 0x30DE, 0x30E8, 0x30F2, 0x30FC, 
0x3106, 0x3110, 0x311A, 0x3124, 0x312E, 
0x3138, 0x3142, 0x314C, 0x3156, 0x3160, 
0x316A, 0x3174, 0x317E, 0x3188, 0x3192, 
0x319C, 0x31A6, 0x31B0, 0x31BA, 0x31C4, 
0x31CE, 0x31D8, 0x31E2, 0x31EC, 0x31F6, 
0x3200, 0x320A, 0x3214, 0x321E, 0x3228, 
0x3232, 0x323C, 0x3246, 0x3250, 0x325A, 
0x3264, 0x326E, 0x3278, 0x3282, 0x328C, 
0x3296, 0x32A0, 0x32AA, 0x32B4, 0x32BE, 
0x32C8, 0x32D2, 0x32DC, 0x32E6, 0x32F0, 
0x32FA, 0x3304, 0x330E, 0x3318, 0x3322, 
0x332C, 0x3336, 0x3340, 0x334A, 0x3354, 
0x335E, 0x3368, 0x3372, 0x337C, 0x3386, 
0x3390, 0x339A, 0x33A4, 0x33AE, 0x33B8, 
0x33C2, 0x33CC, 0x33D6, 0x33E0, 0x33EA, 
0x33F4, 0x33FE, 0x3408, 0x3412, 0x341C, 
0x3426, 0x3430, 0x343A, 0x3444, 0x344E, 
0x3458, 0x3462, 0x346C, 0x3476, 0x3480, 
0x348A, 0x3494, 0x349E, 0x34A8, 0x34B2, 
0x34BC, 0x34C6, 0x34D0, 0x34DA, 0x34E4, 
0x34EE, 0x34F8, 0x3502, 0x350C, 0x3516, 
0x3520, 0x352A, 0x3534, 0x353E, 0x3548, 
0x3552, 0x355C, 0x3566, 0x3570, 0x357A, 
0x3584, 0x358E, 0x3598, 0x35A2, 0x35AC, 
0x35B6, 0x35C0, 0x35CA, 0x35D4, 0x35DE, 
0x35E8, 0x35F2, 0x35FC, 0x3606, 0x3610, 
0x361A, 0x3624, 0x362E, 0x3638, 0x3642, 
0x364C, 0x3656, 0x3660, 0x366A, 0x3674, 
0x367E, 0x3688, 0x3692, 0x369C, 0x36A6, 
0x36B0, 0x36BA, 0x36C4, 0x36CE, 0x36D8, 
0x36E2, 0x36EC, 0x36F6, 0x3700, 0x370A, 
0x3714, 0x371E, 0x3728, 0x3732, 0x373C, 
0x3746, 0x3750, 0x375A, 0x3764, 0x376E, 
0x3778, 0x3782, 0x378C, 0x3796, 0x37A0, 
0x37AA, 0x37B4, 0x37BE, 0x37C8, 0x37D2, 
0x37DC, 0x37E6, 0x37F0, 0x37FA, 0x3804, 
0x380E, 0x3818, 0x3822, 0x382C, 0x3836, 
0x3840, 0x384A, 0x3854, 0x385E, 0x3868, 
0x3872, 0x387C, 0x3886, 0x3890, 0x389A, 
0x38A4, 0x38AE, 0x38B8, 0x38C2, 0x38CC, 
0x38D6, 0x38E0, 0x38EA, 0x38F4, 0x38FE, 
0x3908, 0x3912, 0x391C, 0x3926, 0x3930, 
0x393A, 0x3944, 0x394E, 0x3958, 0x3962, 
0x396C, 0x3976, 0x3980, 0x398A, 0x3994, 
0x399E, 0x39A8, 0x39B2, 0x39BC, 0x39C6, 
0x39D0, 0x39DA, 0x39E4, 0x39EE, 0x39F8, 
0x3A02, 0x3A0C, 0x3A16, 0x3A20, 0x3A2A, 
0x3A34, 0x3A3E, 0x3A48, 0x3A52, 0x3A5C, 
0x3A66, 0x3A70, 0x3A7A, 0x3A84, 0x3A8E, 
0x3A98, 0x3AA2, 0x3AAC, 0x3AB6, 0x3AC0, 
0x3ACA, 0x3AD4, 0x3ADE, 0x3AE8, 0x3AF2, 
0x3AFC, 0x3B06, 0x3B10, 0x3B1A, 0x3B24, 
0x3B2E, 0x3B38, 0x3B42, 0x3B4C, 0x3B56, 
0x3B60, 0x3B6A, 0x3B74, 0x3B7E, 0x3B88, 
0x3B92, 0x3B9C, 0x3BA6, 0x3BB0, 0x3BBA, 
0x3BC4, 0x3BCE, 0x3BD8, 0x3BE2, 0x3BEC, 
0x3BF6, 0x3C00, 0x3C0A, 0x3C14, 0x3C1E, 
0x3C28, 0x3C32, 0x3C3C, 0x3C46, 0x3C50, 
0x3C5A, 0x3C64, 0x3C6E, 0x3C78, 0x3C82, 
0x3C8C, 0x3C96, 0x3CA0, 0x3CAA, 0x3CB4, 
0x3CBE, 0x3CC8, 0x3CD2, 0x3CDC, 0x3CE6, 
0x3CF0, 0x3CFA, 0x3D04, 0x3D0E, 0x3D18, 
0x3D22, 0x3D2C, 0x3D36, 0x3D40, 0x3D4A, 
0x3D54, 0x3D5E, 0x3D68, 0x3D72, 0x3D7C, 
0x3D86, 0x3D90, 0x3D9A, 0x3DA4, 0x3DAE, 
0x3DB8, 0x3DC2, 0x3DCC, 0x3DD6, 0x3DE0, 
0x3DEA, 0x3DF4, 0x3DFE, 0x3E08, 0x3E12, 
0x3E1C, 0x3E26, 0x3E30, 0x3E3A, 0x3E44, 
0x3E4E, 0x3E58, 0x3E62, 0x3E6C, 0x3E76, 
0x3E80, 0x3E8A, 0x3E94, 0x3E9E, 0x3EA8, 
0x3EB2, 0x3EBC, 0x3EC6, 0x3ED0, 0x3EDA, 
0x3EE4, 0x3EEE, 0x3EF8, 0x3F02, 0x3F0C, 
0x3F16, 0x3F20, 0x3F2A, 0x3F34, 0x3F3E, 
0x3F48, 0x3F52, 0x3F5C, 0x3F66, 0x3F70, 
0x3F7A, 0x3F84, 0x3F8E, 0x3F98, 0x3FA2, 
0x3FAC, 0x3FB6, 0x3FC0, 0x3FCA, 0x3FD4, 
0x3FDE, 0x3FE8, 0x3FF2, 0x3FFC, 0x4006, 
0x4010, 0x401A, 0x4024, 0x402E, 0x4038, 
0x4042, 0x404C, 0x4056, 0x4060, 0x406A, 
0x4074, 0x407E, 0x4088, 0x4092, 0x409C, 
0x40A6, 0x40B0, 0x40BA, 0x40C4, 0x40CE, 
0x40D8, 0x40E2, 0x40EC, 0x40F6, 0x4100, 
0x410A, 0x4114, 0x411E, 0x4128, 0x4132, 
0x413C, 0x4146, 0x4150, 0x415A, 0x4164, 
0x416E, 0x4178, 0x4182, 0x418C, 0x4196, 
0x41A0, 0x41AA, 0x41B4, 0x41BE, 0x41C8, 
0x41D2, 0x41DC, 0x41E6, 0x41F0, 0x41FA, 
0x4204, 0x420E, 0x4218, 0x4222, 0x422C, 
0x4236, 0x4240, 0x424A, 0x4254, 0x425E, 
0x4268, 0x4272, 0x427C, 0x4286, 0x4290, 
0x429A, 0x42A4, 0x42AE, 0x42B8, 0x42C2, 
0x42CC, 0x42D6, 0x42E0, 0x42EA, 0x42F4, 
0x42FE, 0x4308, 0x4312, 0x431C, 0x4326, 
0x4330, 0x433A, 0x4344, 0x434E, 0x4358, 
0x4362, 0x436C, 0x4376, 0x4380, 0x438A, 
0x4394, 0x439E, 0x43A8, 0x43B2, 0x43BC, 
0x43C6, 0x43D0, 0x43DA, 0x43E4, 0x43EE, 
0x43F8, 0x4402, 0x440C, 0x4416, 0x4420, 
0x442A, 0x4434, 0x443E, 0x4448, 0x4452, 
0x445C, 0x4466, 0x4470, 0x447A, 0x4484, 
0x448E, 0x4498, 0x44A2, 0x44AC, 0x44B6, 
0x44C0, 0x44CA, 0x44D4, 0x44DE, 0x44E8, 
0x44F2, 0x44FC, 0x4506, 0x4510, 0x451A, 
0x4524, 0x452E, 0x4538, 0x4542, 0x454C, 
0x4556, 0x4560, 0x456A, 0x4574, 0x457E, 
0x4588, 0x4592, 0x459C, 0x45A6, 0x45B0, 
0x45BA, 0x45C4, 0x45CE, 0x45D8, 0x45E2, 
0x45EC, 0x45F6, 0x4600, 0x460A, 0x4614, 
0x461E, 0x4628, 0x4632, 0x463C, 0x4646, 
0x4650, 0x465A, 0x4664, 0x466E, 0x4678, 
0x4682, 0x468C, 0x4696, 0x46A0, 0x46AA, 
0x46B4, 0x46BE, 0x46C8, 0x46D2, 0x46DC, 
0x46E6, 0x46F0, 0x46FA, 0x4704, 0x470E, 
0x4718, 0x4722, 0x472C, 0x4736, 0x4740, 
0x474A, 0x4754, 0x475E, 0x4768, 0x4772, 
0x477C, 0x4786, 0x4790, 0x479A, 0x47A4, 
0x47AE, 0x47B8, 0x47C2, 0x47CC, 0x47D6, 
0x47E0, 0x47EA, 0x47F4, 0x47FE, 0x4808, 
0x4812, 0x481C, 0x4826, 0x4830, 0x483A, 
0x4844, 0x484E, 0x4858, 0x4862, 0x486C, 
0x4876, 0x4880, 0x488A, 0x4894, 0x489E, 
0x48A8, 0x48B2, 0x48BC, 0x48C6, 0x48D0, 
0x48DA, 0x48E4, 0x48EE, 0x48F8, 0x4902, 
0x490C, 0x4916, 0x4920, 0x492A, 0x4934, 
0x493E, 0x4948, 0x4952, 0x495C, 0x4966, 
0x4970, 0x497A, 0x4984, 0x498E, 0x4998, 
0x49A2, 0x49AC, 0x49B6, 0x49C0, 0x49CA, 
0x49D4, 0x49DE, 0x49E8, 0x49F2, 0x49FC, 
0x4A06, 0x4A10, 0x4A1A, 0x4A24, 0x4A2E, 
0x4A38, 0x4A42, 0x4A4C, 0x4A56, 0x4A60, 
0x4A6A, 0x4A74, 0x4A7E, 0x4A88, 0x4A92, 
0x4A9C, 0x4AA6, 0x4AB0, 0x4ABA, 0x4AC4, 
0x4ACE, 0x4AD8, 0x4AE2, 0x4AEC, 0x4AF6, 
0x4B00, 0x4B0A, 0x4B14, 0x4B1E, 0x4B28, 
0x4B32, 0x4B3C, 0x4B46, 0x4B50, 0x4B5A, 
0x4B64, 0x4B6E, 0x4B78, 0x4B82, 0x4B8C, 
0x4B96, 0x4BA0, 0x4BAA, 0x4BB4, 0x4BBE, 
0x4BC8, 0x4BD2, 0x4BDC, 0x4BE6, 0x4BF0, 
0x4BFA, 0x4C04, 0x4C0E, 0x4C18, 0x4C22, 
0x4C2C, 0x4C36, 0x4C40, 0x4C4A, 0x4C54, 
0x4C5E, 0x4C68, 0x4C72, 0x4C7C, 0x4C86, 
0x4C90, 0x4C9A, 0x4CA4, 0x4CAE, 0x4CB8, 
0x4CC2, 0x4CCC, 0x4CD6, 0x4CE0, 0x4CEA, 
0x4CF4, 0x4CFE, 0x4D08, 0x4D12, 0x4D1C, 
0x4D26, 0x4D30, 0x4D3A, 0x4D44, 0x4D4E, 
0x4D58, 0x4D62, 0x4D6C, 0x4D76, 0x4D80, 
0x4D8A, 0x4D94, 0x4D9E, 0x4DA8, 0x4DB2, 
0x4DBC, 0x4DC6, 0x4DD0, 0x4DDA, 0x4DE4, 
0x4DEE, 0x4DF8, 0x4E02, 0x4E0C, 0x4E16, 
0x4E20, 0x4E2A, 0x4E34, 0x4E3E, 0x4E48, 
0x4E52, 0x4E5C, 0x4E66, 0x4E70, 0x4E7A, 
0x4E84, 0x4E8E, 0x4E98, 0x4EA2, 0x4EAC, 
0x4EB6, 0x4EC0, 0x4ECA, 0x4ED4, 0x4EDE, 
0x4EE8, 0x4EF2, 0x4EFC, 0x4F06, 0x4F10, 
0x4F1A, 0x4F24, 0x4F2E, 0x4F38, 0x4F42, 
0x4F4C, 0x4F56, 0x4F60, 0x4F6A, 0x4F74, 
0x4F7E, 0x4F88, 0x4F92, 0x4F9C, 0x4FA6, 
0x4FB0, 0x4FBA, 0x4FC4, 0x4FCE, 0x4FD8, 
0x4FE2, 0x4FEC, 0x4FF6, 0x5000, 0x500A, 
0x5014, 0x501E, 0x5028, 0x5032, 0x503C, 
0x5046, 0x5050, 0x505A, 0x5064, 0x506E, 
0x5078, 0x5082, 0x508C, 0x5096, 0x50A0, 
0x50AA, 0x50B4, 0x50BE, 0x50C8, 0x50D2, 
0x50DC, 0x50E6, 0x50F0, 0x50FA, 0x5104, 
0x510E, 0x5118, 0x5122, 0x512C, 0x5136, 
0x5140, 0x514A, 0x5154, 0x515E, 0x5168, 
0x5172, 0x517C, 0x5186, 0x5190, 0x519A, 
0x51A4, 0x51AE, 0x51B8, 0x51C2, 0x51CC, 
0x51D6, 0x51E0, 0x51EA, 0x51F4, 0x51FE, 
0x5208, 0x5212, 0x521C, 0x5226, 0x5230, 
0x523A, 0x5244, 0x524E, 0x5258, 0x5262, 
0x526C, 0x5276, 0x5280, 0x528A, 0x5294, 
0x529E, 0x52A8, 0x52B2, 0x52BC, 0x52C6, 
0x52D0, 0x52DA, 0x52E4, 0x52EE, 0x52F8, 
0x5302, 0x530C, 0x5316, 0x5320, 0x532A, 
0x5334, 0x533E, 0x5348, 0x5352, 0x535C, 
0x5366, 0x5370, 0x537A, 0x5384, 0x538E, 
0x5398, 0x53A2, 0x53AC, 0x53B6, 0x53C0, 
0x53CA, 0x53D4, 0x53DE, 0x53E8, 0x53F2, 
0x53FC, 0x5406, 0x5410, 0x541A, 0x5424, 
0x542E, 0x5438, 0x5442, 0x544C, 0x5456, 
0x5460, 0x546A, 0x5474, 0x547E, 0x5488, 
0x5492, 0x549C, 0x54A6, 0x54B0, 0x54BA, 
0x54C4, 0x54CE, 0x54D8, 0x54E2, 0x54EC, 
0x54F6, 0x5500, 0x550A, 0x5514, 0x551E, 
0x5528, 0x5532, 0x553C, 0x5546, 0x5550, 
0x555A, 0x5564, 0x556E, 0x5578, 0x5582, 
0x558C, 0x5596, 0x55A0, 0x55AA, 0x55B4, 
0x55BE, 0x55C8, 0x55D2, 0x55DC, 0x55E6, 
0x55F0, 0x55FA, 0x5604, 0x560E, 0x5618, 
0x5622, 0x562C, 0x5636, 0x5640, 0x564A, 
0x5654, 0x565E, 0x5668, 0x5672, 0x567C, 
0x5686, 0x5690, 0x569A, 0x56A4, 0x56AE, 
0x56B8, 0x56C2, 0x56CC, 0x56D6, 0x56E0, 
0x56EA, 0x56F4, 0x56FE, 0x5708, 0x5712, 
0x571C, 0x5726, 0x5730, 0x573A, 0x5744, 
0x574E, 0x5758, 0x5762, 0x576C, 0x5776, 
0x5780, 0x578A, 0x5794, 0x579E, 0x57A8, 
0x57B2, 0x57BC, 0x57C6, 0x57D0, 0x57DA, 
0x57E4, 0x57EE, 0x57F8, 0x5802, 0x580C, 
0x5816, 0x5820, 0x582A, 0x5834, 0x583E, 
0x5848, 0x5852, 0x585C, 0x5866, 0x5870, 
0x587A, 0x5884, 0x588E, 0x5898, 0x58A2, 
0x58AC, 0x58B6, 0x58C0, 0x58CA, 0x58D4, 
0x58DE, 0x58E8, 0x58F2, 0x58FC, 0x5906, 
0x5910, 0x591A, 0x5924, 0x592E, 0x5938, 
0x5942, 0x594C, 0x5956, 0x5960, 0x596A, 
0x5974, 0x597E, 0x5988, 0x5992, 0x599C, 
0x59A6, 0x59B0, 0x59BA, 0x59C4, 0x59CE, 
0x59D8
};

#endif  // ETC_CRUVE_H
