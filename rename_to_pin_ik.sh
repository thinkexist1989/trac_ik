#!/bin/bash
# 重命名 TRAC_IK 到 PIN_IK

set -e

echo "开始重命名 TRAC_IK -> PIN_IK..."

# 1. 重命名源文件和目录
echo "1. 重命名文件和目录..."

# 重命名主库目录
if [ -d "trac_ik_lib" ]; then
    git mv trac_ik_lib pin_ik_lib
fi

# 重命名示例目录
if [ -d "trac_ik_examples" ]; then
    git mv trac_ik_examples pin_ik_examples
fi

# 重命名 Python 目录
if [ -d "trac_ik_python" ]; then
    git mv trac_ik_python pin_ik_python
fi

# 重命名 CMake 配置文件
if [ -f "cmake/trac_ikConfig.cmake.in" ]; then
    git mv cmake/trac_ikConfig.cmake.in cmake/pin_ikConfig.cmake.in
fi

# 重命名头文件
if [ -f "pin_ik_lib/include/trac_ik/trac_ik.hpp" ]; then
    mkdir -p pin_ik_lib/include/pin_ik
    git mv pin_ik_lib/include/trac_ik/trac_ik.hpp pin_ik_lib/include/pin_ik/pin_ik.hpp
    git mv pin_ik_lib/include/trac_ik/nlopt_ik.hpp pin_ik_lib/include/pin_ik/nlopt_ik.hpp
    git mv pin_ik_lib/include/trac_ik/pinocchio_tl.hpp pin_ik_lib/include/pin_ik/pinocchio_tl.hpp
    git mv pin_ik_lib/include/trac_ik/pinocchio_types.hpp pin_ik_lib/include/pin_ik/pinocchio_types.hpp
    git mv pin_ik_lib/include/trac_ik/urdf.hpp pin_ik_lib/include/pin_ik/urdf.hpp
    rmdir pin_ik_lib/include/trac_ik 2>/dev/null || true
fi

# 重命名源文件
if [ -f "pin_ik_lib/src/trac_ik.cpp" ]; then
    git mv pin_ik_lib/src/trac_ik.cpp pin_ik_lib/src/pin_ik.cpp
fi

# 重命名 SWIG 文件
if [ -f "pin_ik_python/swig/trac_ik_wrap.i" ]; then
    git mv pin_ik_python/swig/trac_ik_wrap.i pin_ik_python/swig/pin_ik_wrap.i
fi

echo "文件和目录重命名完成"

# 2. 替换文件内容中的文本
echo "2. 替换文件内容..."

# 查找所有需要修改的文件（排除 .git 和 build）
FILES=$(find . -type f \( -name "*.hpp" -o -name "*.cpp" -o -name "*.txt" -o -name "*.cmake" -o -name "*.in" -o -name "*.i" -o -name "*.py" -o -name "*.md" \) | grep -v ".git" | grep -v build | grep -v ".swp")

for file in $FILES; do
    # 跳过二进制文件
    if file "$file" | grep -q "text"; then
        # 替换命名空间和类名
        sed -i 's/TRAC_IK/PIN_IK/g' "$file"
        sed -i 's/trac_ik/pin_ik/g' "$file"
        sed -i 's/TRAC-IK/PIN-IK/g' "$file"
        sed -i 's/trac-ik/pin-ik/g' "$file"
        # 保留 trac_ik 在 URL 和引用中（如果需要）
    fi
done

echo "文件内容替换完成"

echo "重命名完成！请运行以下命令验证："
echo "  git status"
echo "  git diff --cached"
