#!/bin/bash

echo "📦 Instalando Unitree SDK2 manualmente..."

SDK_SRC="/home/pedro_setubal/Workspaces/G1/unitree_sdk2"
SDK_BUILD="$SDK_SRC/build"
INSTALL_PREFIX="/opt/unitree_robotics"

# Criar diretórios
echo "Criando diretórios..."
sudo mkdir -p $INSTALL_PREFIX/lib/cmake/unitree_sdk2
sudo mkdir -p $INSTALL_PREFIX/include
sudo mkdir -p $INSTALL_PREFIX/lib/x86_64

# Copiar arquivos cmake
echo "Copiando arquivos cmake..."
sudo cp $SDK_BUILD/unitree_sdk2*.cmake $INSTALL_PREFIX/lib/cmake/unitree_sdk2/

# Copiar headers
echo "Copiando headers..."
sudo cp -r $SDK_SRC/include/* $INSTALL_PREFIX/include/

# Copiar bibliotecas
echo "Copiando bibliotecas..."
if [ -d "$SDK_SRC/lib/x86_64" ]; then
    sudo cp -r $SDK_SRC/lib/x86_64/* $INSTALL_PREFIX/lib/x86_64/
fi

# Copiar thirdparty libs
echo "Copiando bibliotecas third-party..."
if [ -d "$SDK_SRC/thirdparty/lib/x86_64" ]; then
    sudo mkdir -p $INSTALL_PREFIX/thirdparty/lib/x86_64
    sudo cp -r $SDK_SRC/thirdparty/lib/x86_64/* $INSTALL_PREFIX/thirdparty/lib/x86_64/
fi

echo "✅ Instalação manual concluída!"
echo ""
echo "Agora tente compilar o unitree_mujoco:"
echo "cd ~/Workspaces/G1/unitree_mujoco/simulate/build"
echo "cmake .. -DCMAKE_BUILD_TYPE=Release"
echo "make -j\$(nproc)"