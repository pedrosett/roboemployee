#!/bin/bash

echo "🔍 Verificação do Setup G1 com Mãos Completas"
echo "=============================================="

# Cores
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Função para verificar arquivo
check_file() {
    if [ -f "$1" ]; then
        echo -e "${GREEN}✅${NC} $2"
        return 0
    else
        echo -e "${RED}❌${NC} $2 - NÃO ENCONTRADO"
        return 1
    fi
}

# Função para verificar diretório
check_dir() {
    if [ -d "$1" ]; then
        echo -e "${GREEN}✅${NC} $2"
        return 0
    else
        echo -e "${RED}❌${NC} $2 - NÃO ENCONTRADO"
        return 1
    fi
}

echo -e "\n1. Verificando MuJoCo:"
check_file "/usr/local/bin/simulate" "MuJoCo instalado"
check_file "/usr/local/lib/libmujoco.so" "Biblioteca MuJoCo"

echo -e "\n2. Verificando Unitree SDK2:"
check_dir "/opt/unitree_robotics" "Unitree SDK2 instalado"

echo -e "\n3. Verificando arquivos do G1:"
G1_DIR="/home/pedro_setubal/Workspaces/G1/unitree_mujoco/unitree_robots/g1"
check_file "$G1_DIR/g1_29dof_with_hand.xml" "Modelo G1 com mãos completas"
check_file "$G1_DIR/scene_29dof_with_hand.xml" "Cena G1 com mãos"

echo -e "\n4. Verificando meshes das mãos:"
MESHES=(
    "left_hand_thumb_0_link.STL"
    "left_hand_thumb_1_link.STL"
    "left_hand_thumb_2_link.STL"
    "left_hand_index_0_link.STL"
    "left_hand_index_1_link.STL"
    "left_hand_middle_0_link.STL"
    "left_hand_middle_1_link.STL"
    "right_hand_thumb_0_link.STL"
    "right_hand_thumb_1_link.STL"
    "right_hand_thumb_2_link.STL"
    "right_hand_index_0_link.STL"
    "right_hand_index_1_link.STL"
    "right_hand_middle_0_link.STL"
    "right_hand_middle_1_link.STL"
)

all_meshes=true
for mesh in "${MESHES[@]}"; do
    if ! check_file "$G1_DIR/meshes/$mesh" "  $mesh"; then
        all_meshes=false
    fi
done

echo -e "\n5. Verificando simulador:"
SIM_DIR="/home/pedro_setubal/Workspaces/G1/unitree_mujoco/simulate"
check_dir "$SIM_DIR" "Diretório simulate"
check_file "$SIM_DIR/CMakeLists.txt" "CMakeLists.txt"

if [ -f "$SIM_DIR/build/unitree_mujoco" ]; then
    echo -e "${GREEN}✅${NC} Simulador compilado"
    echo "   Para executar:"
    echo "   cd $SIM_DIR/build"
    echo "   ./unitree_mujoco -c ../config_g1_hands_test.yaml"
else
    echo -e "${RED}❌${NC} Simulador não compilado"
    echo "   Para compilar:"
    echo "   cd $SIM_DIR"
    echo "   mkdir -p build && cd build"
    echo "   cmake .. -DCMAKE_BUILD_TYPE=Release"
    echo "   make -j\$(nproc)"
fi

echo -e "\n6. Análise do modelo G1:"
if [ -f "$G1_DIR/g1_29dof_with_hand.xml" ]; then
    echo "Contando elementos no modelo:"
    echo -n "  Meshes de mãos: "
    grep -c "hand.*link.STL" "$G1_DIR/g1_29dof_with_hand.xml"
    echo -n "  Total de joints: "
    grep -c "<joint" "$G1_DIR/g1_29dof_with_hand.xml"
    echo -n "  Joints de mãos: "
    grep -E "hand.*joint" "$G1_DIR/g1_29dof_with_hand.xml" | wc -l
fi

echo -e "\n=============================================="
if $all_meshes && [ -f "$G1_DIR/g1_29dof_with_hand.xml" ]; then
    echo -e "${GREEN}✅ G1 com mãos completas está pronto!${NC}"
else
    echo -e "${RED}❌ Alguns componentes estão faltando${NC}"
fi