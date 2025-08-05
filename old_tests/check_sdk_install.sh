#!/bin/bash

echo "🔍 Verificando instalação do Unitree SDK2"
echo "========================================="

# Verificar diretórios de instalação
echo -e "\n1. Verificando diretórios padrão:"
for dir in "/opt/unitree_robotics" "/usr/local/lib/cmake/unitree_sdk2" "/opt/unitree_robotics/lib/cmake/unitree_sdk2"; do
    if [ -d "$dir" ]; then
        echo "✅ Encontrado: $dir"
        ls -la "$dir" 2>/dev/null | head -5
    else
        echo "❌ Não encontrado: $dir"
    fi
done

echo -e "\n2. Procurando arquivos cmake do SDK:"
find /opt -name "*unitree_sdk2*.cmake" 2>/dev/null
find /usr/local -name "*unitree_sdk2*.cmake" 2>/dev/null

echo -e "\n3. Verificando biblioteca instalada:"
find /opt -name "libunitree_sdk2*" 2>/dev/null
find /usr/local -name "libunitree_sdk2*" 2>/dev/null

echo -e "\n4. Solução alternativa - Compilar sem SDK2:"
echo "Se o SDK não estiver instalado corretamente, você pode:"
echo "1. Instalar manualmente:"
echo "   cd ~/Workspaces/G1/unitree_sdk2/build"
echo "   sudo make install"
echo ""
echo "2. Ou tentar compilar sem o SDK (funcionalidade limitada):"
echo "   Edite ~/Workspaces/G1/unitree_mujoco/simulate/CMakeLists.txt"
echo "   Comente a linha: find_package(unitree_sdk2 REQUIRED)"
echo "   Remova unitree_sdk2 das dependências"