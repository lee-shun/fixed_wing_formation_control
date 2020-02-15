/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-02-14 18:35:01
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  滤波器测试文件（函数）
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors  : lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime : 2020-02-15 19:07:18
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#include <iostream>
#include "../fixed_wing_lib/filter.hpp"

using namespace std;

int main(void)
{
    FILTER test_filter;

    for (int i = 0; i <= 100; i++)
    {
        cout << "test_filter ===" << test_filter.one_order_filter(1) << endl;
    }
    return 0;
}
