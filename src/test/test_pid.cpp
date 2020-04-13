/*
 * @------------------------------------------1: 1------------------------------------------@
 * @Author: lee-shun
 * @Email: 2015097272@qq.com
 * @Date: 2020-04-10 23:44:40
 * @Organization: BIT-CGNC, fixed_wing_group
 * @Description:  测试PID控制器
 * @------------------------------------------2: 2------------------------------------------@
 * @LastEditors: lee-shun
 * @LastEditors_Email: 2015097272@qq.com
 * @LastEditTime: 2020-04-11 23:04:56
 * @LastEditors_Organization: BIT-CGNC, fixed_wing_group
 * @LastEditors_Description:  
 * @------------------------------------------3: 3------------------------------------------@
 */
#include "../fixed_wing_lib/increment_pid_controller.hpp"
#include <iostream>

using namespace std;

int main()
{
  INCREMENT_PID_CONTROLLER pid;
  int i = 0;
  for (i = 1; i < 10; i++)
  {
    /* 请注意static关键字的用发，若是初始化之后便不再初始化，但是后面可以更改其值，利用初始化的方式不能更改 */
    static float increment;
    pid.increment_pid(i, 1, 1, 1);
    cout << "i" << i << endl;
    cout << "increment" << increment << endl;
  }
  return 0;
}
