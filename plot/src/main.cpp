// 2023-11-29

#include "myodom.h"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<wx::CMyOdom>());
    rclcpp::shutdown();

    return 0;
}

/*
 * 输出完全精度

您可以直接在std::cout 上设置精度并使用 std::fixed 格式说明符。

double d = 3.14159265358979;
cout.precision(17);
cout << "Pi: " << fixed << d << endl;
您可以 #include <limits>获得 float 或 double 的最大精度。

#include <limits>

typedef std::numeric_limits< double > dbl;

double d = 3.14159265358979;
cout.precision(dbl::max_digits10);
cout << "Pi: " << d << endl;

// ---

using namespace std;

int main(){
	double value = 12.3456789;

	// 默认精度是6，所以输出为 12.3457 
	//（默认情况下，精度是指总的有效数字）
	cout << value << endl; 

	// 把精度修改为4， 输出12.35, 对最后一位四舍五入
	// 精度修改后，持续有效，直到精度再次被修改
	cout.precision(4);
	cout << value << endl;

	// 使用定点法， 精度变成小数点后面的位数
    // 输出12.3457
	cout.flags(cout.fixed); 
	cout  << value << endl;

	// 定点法持续有效
    // 输出3.1416
	cout << 3.1415926535 << endl;

	// 把精度恢复成有效数字位数
	cout.unsetf(cout.fixed);
	cout  << value << endl;          //输出12.35
	cout << 3.1415926535 << endl;  //输出3.142

	system("pause");
	return 0;
}
*/

/*
 * 关于double类型的数据设置精度输出，没有验证
    std::cout << std::setprecision (15) << 3.14159265358979 << std::endl;
    std::cout << std::setprecision (std::numeric_limits<double>::digits10 + 1)
    << 3.14159265358979
    << std::endl;

    double d = 3.14159265358979;
    std::cout.precision(17);
    std::cout << "Pi: " << std::fixed << d << endl;

    #include <limits>

    typedef std::numeric_limits< double > dbl;

    double d = 3.14159265358979;
    cout.precision(dbl::max_digits10);
    cout << "Pi: " << d << endl;
 *
 */