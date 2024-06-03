/*!
 * @file utilities.h
 * @brief Common utility functions
 */

#ifndef PROJECT_UTILITIES_H
#define PROJECT_UTILITIES_H

#include <algorithm>
#include <map>
#include <random>
#include <unordered_map>
#include <vector>
#include <iomanip>
#include "cpp_types.hpp"

/**

    @brief 该文件包含通用实用函数
*/

/********** 排序工具一：数值比较 **********/

/**

    @brief 判断两个浮点数是否几乎相等
    @param a 第一个值
    @param b 第二个值
    @param tol 相等的容差
    @return true 相等
    @return false 不相等
*/
template < typename T > bool IsNumbersEqual( T a, T b, T tol ) {
    return std::abs( a - b ) <= tol;
}

/**

    @brief 判断两个向量是否相等，使用 "!=" 运算符进行比较。
    @param a 第一个向量
    @param b 第二个向量
    @return true 相等
    @return false 不相等
*/
template < typename T > bool IsVectorEqual( const std::vector< T >& a, const std::vector< T >& b ) {
    if ( a.size() != b.size() )
        return false;
    for ( size_t i = 0; i < a.size(); i++ ) {
        if ( a[ i ] != b[ i ] )
            return false;
    }
    return true;
}


/**

    @brief 使输入值位于指定范围[min, max]内
    @param target 输入值
    @param min 范围的最小值
    @param max 范围的最大值
    @return T
*/
template < typename T > T WrapRange( const T& target, const T& min, const T& max ) {
    T result = target;
    assert( min <= max );
    if ( result < min ) {
        result = min;
    }
    if ( max < result ) {
        result = max;
    }
    return result;
}

/**

    @brief 应用死区
    @param x 输入值
    @param range 死区（0周围的范围）
    @return T
*/
template < typename T > T ApplyDeadband( T x, T range ) {
    if ( x < range && x > -range )
        x = T( 0 );
    return x;
}

/**

    @brief 应用死区并将输入值强制限制在最小值和最大值之间
    @param x 输入值
    @param range 死区（0周围的范围）
    @param min 最小值
    @param max 最大值
    @return T
*/
template < typename T > T ApplyDeadband( T x, T range, T min, T max ) {
    if ( x < range && x > -range )
        x = T( 0 );
    return std::min( std::max( x, min ), max );
}

/**
 * @brief 对Eigen矩阵应用死区
 *
 * @param v 输入矩阵
 * @param band 死区（0周围的范围）
 */
template < typename T > void EigenApplyDeadband( Eigen::MatrixBase< T >& v, typename T::Scalar band ) {
    for ( size_t i = 0; i < T::RowsAtCompileTime; i++ ) {
        for ( size_t j = 0; j < T::ColsAtCompileTime; j++ ) {
            v( i, j ) = ApplyDeadband( v( i, j ), band );
        }
    }
}

/**
 * @brief 获取数字的符号
 *
 * @param val 输入值
 * @return int 1 表示正数，0 表示0，-1 表示负数
 */
template < typename T > int MathSign( T val ) {
    return ( T( 0 ) < val ) - ( val < T( 0 ) );
}


/**
 * @brief 判断无序映射是否包含给定元素
 *
 * @param set 输入映射
 * @param key 比较的元素
 * @return true 包含
 * @return false 不包含
 */
template < typename T1, typename T2 > bool DoesMapContains( const std::unordered_map< T1, T2 >& set, T1 key ) {
    return set.find( key ) != set.end();
}

/**
 * @brief 判断映射是否包含给定元素
 *
 * @param set 输入映射
 * @param key 比较的元素
 * @return true 包含
 * @return false 不包含
 */
template < typename T1, typename T2 > bool DoesMapContains( const std::map< T1, T2 >& set, T1 key ) {
    return set.find( key ) != set.end();
}

/********** 排序工具二：数据类型转换 **********/

/**
 * @brief 将浮点数转换为字符串。与 std::to_string 相比更可取，因为这使用科学计数法，并不会截断小数/大数。
 *
 * @param number 浮点数
 * @return std::string
 */
template < typename T > std::string NumberToString( T number ) {
    static_assert( std::is_floating_point< T >::value, "numberToString must use a floating point type!" );
    char buffer[ 100 ];
    sprintf( buffer, "%g", number );
    return std::string( buffer );
}

/*!
 * 将值 x 在 (inputMin, inputMax) 范围内线性映射到 (outputMin, outputMax) 范围内
 */
template <typename T>
T mapToRange(T x, T inputMin, T inputMax, T outputMin, T outputMax) {
  return outputMin +
         (x - inputMin) * (outputMax - outputMin) / (inputMax - inputMin);
}

/**
 * @brief 将 Eigen 类型转换为 std::string。
 *
 * @param value 输入的 Eigen 类型数据
 * @return std::string
 */
template < typename T > std::string EigenToString( Eigen::MatrixBase< T >& value ) {
    std::stringstream ss;
    ss << value;
    return ss.str();
}

/**
 * @brief 将布尔值转换为字符串（true、false）
 *
 * @param b 布尔类型的输入数据
 * @return std::string
 */
static inline std::string BoolToString( bool b ) {
    return std::string( b ? "true" : "false" );
}

/**
 * @brief 将字符串转换为浮点数或双精度浮点数。
 *
 * @param str 字符串类型的输入数据
 * @return T
 */
template < typename T > T StringToNumber( const std::string& str ) {
    static_assert( std::is_same< T, double >::value || std::is_same< T, float >::value, "StringToNumber only works for double/float" );

    if ( std::is_same< T, double >::value ) {
        return std::stod( str );
    }
    else if ( std::is_same< T, float >::value ) {
        return std::stof( str );
    }
}

/**
 * @brief 将字符数组转换为浮点数或双精度浮点数。
 *
 * @param str 字符数组类型的输入数据
 * @return T
 */
template < typename T > T StringToNumber( const char* str ) {
    return StringToNumber< T >( std::string( str ) );
}

/**
 * @brief 将字符串转换为 Eigen 矩阵。
 *
 * @param str 字符串类型的输入数据
 * @return Eigen::Matrix< T, Rows, Cols >
 */
template < typename T, int Rows, int Cols > Eigen::Matrix< T, Rows, Cols > StringToEigen( const std::string& str ) {
    Eigen::Matrix< T, Rows, Cols > m;
    size_t                         pos = 0;

    // seek past whitespace
    while ( str.at( pos ) == ' ' )
        pos++;
    if ( str.at( pos ) == '[' ) {
        pos++;
    }
    else {
        throw std::runtime_error( "StringToEigen didn't find open bracket" );
    }
    // the user will write matrix in row major
    for ( int i = 0; i < Rows; i++ ) {
        for ( int j = 0; j < Cols; j++ ) {
            // seek past whitespace
            while ( str.at( pos ) == ' ' )
                pos++;
            size_t start = pos;

            // seek to end of first number
            while ( str.at( pos ) != ',' && str.at( pos ) != ']' )
                pos++;
            m( i, j ) = StringToNumber< T >( str.substr( start, pos - start ) );
            pos++;
        }
    }
    return m;
}


/**
 * @brief 以 printf 样式打印到字符串中。
 *
 * @param format 输入的字符串格式
 * @param args 要打印的变量
 * @return std::string
 */
template < typename... Args > std::string StringFormat( const std::string& format, Args... args ) {
    int size = snprintf( nullptr, 0, format.c_str(), args... ) + 1;  // Extra space for '\0'
    if ( size <= 0 ) {
        throw std::runtime_error( "Error during formatting." );
    }
    std::unique_ptr< char[] > buf( new char[ size ] );
    snprintf( buf.get(), size, format.c_str(), args... );
    return std::string( buf.get(), buf.get() + size - 1 );  // We don't want the '\0' inside
}

/**
 * @brief 检查二维向量的大小
 *
 * @param arr 二维向量输入数据
 * @param rows 检查的行数
 * @param cols 检查的列数
 * @return true
 * @return false
 */
template < typename T > bool Check2DArraySize( const std::vector< std::vector< T > >& arr, int rows, int cols ) {
    if ( ( int )arr.size() != rows ) {
        return false;
    }
    for ( int i = 0; i < rows; i++ ) {
        if ( ( int )arr[ i ].size() != cols ) {
            return false;
        }
    }
    return true;
}

#endif  // PROJECT_UTILITIES_H
