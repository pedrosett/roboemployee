#!/bin/bash

echo "🔍 Verificando instalação do MuJoCo"
echo "===================================="

# Função para verificar comando
check_command() {
    if command -v $1 &> /dev/null; then
        echo "✅ $1 encontrado: $(which $1)"
        return 0
    else
        echo "❌ $1 NÃO encontrado"
        return 1
    fi
}

# Função para verificar arquivo
check_file() {
    if [ -f "$1" ]; then
        echo "✅ $2 encontrado"
        return 0
    else
        echo "❌ $2 NÃO encontrado em $1"
        return 1
    fi
}

# Função para verificar diretório
check_dir() {
    if [ -d "$1" ]; then
        echo "✅ $2 encontrado"
        return 0
    else
        echo "❌ $2 NÃO encontrado em $1"
        return 1
    fi
}

echo -e "\n1. Verificando binários:"
check_command simulate
check_command mjx_simulate

echo -e "\n2. Verificando bibliotecas:"
check_file "/usr/local/lib/libmujoco.so" "libmujoco.so"
check_file "/usr/local/lib/libmujoco.so.3.2.7" "libmujoco.so.3.2.7"

echo -e "\n3. Verificando headers:"
check_dir "/usr/local/include/mujoco" "Headers do MuJoCo"

echo -e "\n4. Verificando cmake:"
if [ -d "/usr/local/share/cmake/mujoco" ] || [ -d "/usr/local/lib/cmake/mujoco" ]; then
    echo "✅ Arquivos cmake do MuJoCo encontrados"
    if [ -f "/usr/local/share/cmake/mujoco/mujocoConfig.cmake" ]; then
        echo "   Config: /usr/local/share/cmake/mujoco/mujocoConfig.cmake"
    fi
    if [ -f "/usr/local/lib/cmake/mujoco/mujocoConfig.cmake" ]; then
        echo "   Config: /usr/local/lib/cmake/mujoco/mujocoConfig.cmake"
    fi
else
    echo "❌ Arquivos cmake do MuJoCo NÃO encontrados"
fi

echo -e "\n5. Verificando pkg-config:"
if pkg-config --exists mujoco 2>/dev/null; then
    echo "✅ pkg-config para MuJoCo configurado"
    echo "   Versão: $(pkg-config --modversion mujoco)"
    echo "   CFLAGS: $(pkg-config --cflags mujoco)"
    echo "   LIBS: $(pkg-config --libs mujoco)"
else
    echo "❌ pkg-config para MuJoCo NÃO configurado"
fi

echo -e "\n6. Teste de linkagem:"
if ldconfig -p | grep -q libmujoco; then
    echo "✅ libmujoco está no cache do ldconfig"
    ldconfig -p | grep libmujoco | head -3
else
    echo "❌ libmujoco NÃO está no cache do ldconfig"
    echo "   Execute: sudo ldconfig"
fi

echo -e "\n===================================="
echo "📋 Status da instalação:"

if command -v simulate &> /dev/null && [ -f "/usr/local/lib/libmujoco.so" ]; then
    echo "✅ MuJoCo parece estar instalado corretamente!"
    echo ""
    echo "Para compilar unitree_mujoco:"
    echo "cd ~/Workspaces/G1/unitree_mujoco/simulate/build"
    echo "cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH=/usr/local"
    echo "make -j\$(nproc)"
else
    echo "❌ MuJoCo NÃO está instalado corretamente!"
    echo ""
    echo "Para instalar:"
    echo "cd ~/Workspaces/G1"
    echo "git clone https://github.com/google-deepmind/mujoco.git"
    echo "cd mujoco && git checkout tags/3.2.7"
    echo "mkdir build && cd build"
    echo "cmake .. -DCMAKE_BUILD_TYPE=Release"
    echo "make -j\$(nproc)"
    echo "sudo make install"
    echo "sudo ldconfig"
fi