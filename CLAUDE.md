# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

🔄 **NOVA FASE DO PROJETO** - Transição da simulação **MuJoCo** (concluída com sucesso) para **Gazebo ROS2** para desenvolvimento avançado de controle robótico do **Unitree G1** com mãos de 5 dedos em ambiente 3D real.

## Status do Projeto (05/08/2024) - 🚧 MIGRAÇÃO MUJOCO → GAZEBO ROS2

### ✅ **FASE MUJOCO CONCLUÍDA (ARQUIVADA):**
- **Simulação MuJoCo 100% funcional** arquivada em [`old_tests/`](old_tests/)
- **G1 com 29 DOF + mãos de 5 dedos** funcionando perfeitamente
- **Ambiente 3D escritório** (33m x 28m x 3.15m) integrado com escala correta
- **Física realista** e controles responsivos validados

### 🚧 **FASE ATUAL - GAZEBO ROS2:**
**Objetivo**: Migrar para Gazebo com integração ROS2 Jazzy para:
1. **Controle ROS2 nativo** via tópicos/serviços/actions
2. **Sensores avançados** (IMU, câmeras, sensores táteis)
3. **Algoritmos de controle** bípede e manipulação
4. **Navegação autônoma** no ambiente escaneado

### 🎯 **PRÓXIMOS PASSOS PRIORITÁRIOS:**
1. **Setup Gazebo ROS2 workspace**
2. **Adaptar URDF G1** para Gazebo (já disponível em `unitree_ros/`)
3. **Converter ambiente 3D** OBJ → SDF/World para Gazebo
4. **Implementar controllers ROS2** para o G1

## Estrutura do Projeto Atual

```
~/Workspaces/G1/
├── README.md                     # 📋 Documentação principal (Gazebo ROS2)
├── CLAUDE.md                     # 🤖 Este arquivo de instruções
├── old_tests/                    # 📦 SIMULAÇÕES MUJOCO ARQUIVADAS
│   ├── mujoco_simulations/       # Arquivos XML, scripts, launchers
│   ├── mujoco_assets/           # Meshes, URDF G1 5-dedos
│   ├── mujoco_source/           # Código fonte MuJoCo 3.2.7
│   └── README.md                # Documentação da fase MuJoCo
├── unitree_ros/                  # 🤖 Pacotes ROS2 oficiais Unitree
│   └── robots/g1_description/    # **URDF + MESHES G1 5 DEDOS**
├── 3d escritorio/                # 🏢 Assets 3D do ambiente LiDAR
│   ├── escritorio_CW_scan.obj    # Scan original (131 grupos)
│   └── body1_structure/          # Paredes extraídas para MuJoCo
├── unitree_guide/                # 📚 Documentação Unitree ROS
├── unitree_sdk2/                 # 🔧 SDK C++ Unitree  
├── unitree_sdk2_python/          # 🐍 Interface Python SDK
└── [NOVOS DIRETÓRIOS A CRIAR]:
    ├── gazebo_workspace/         # Workspace ROS2 para Gazebo
    ├── g1_gazebo_simulation/     # Pacote principal simulação
    └── docs/                     # Documentação técnica
```

## 🚀 **COMANDOS DE REFERÊNCIA - FASE MUJOCO (ARQUIVADA):**

**⚠️ IMPORTANTE**: Estes comandos são para referência da fase anterior:

### Simulação G1 5-dedos (MuJoCo):
```bash
# Comando direto (ainda funciona):
cd ~/Workspaces/G1/unitree_ros/robots/g1_description
simulate g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf

# Ou via launcher arquivado:
cd ~/Workspaces/G1/old_tests/mujoco_simulations/launchers
./launch_g1_5fingers_final.sh
```

## 🛠️ **FERRAMENTAS DISPONÍVEIS - NOVA FASE:**

### ROS2 Jazzy (✅ Configurado):
```bash
source /opt/ros/jazzy/setup.bash  # Já no ~/.bashrc
```

### MuJoCo PATH (✅ Configurado):
```bash
export PATH="/usr/local/bin:$PATH"  # Já no ~/.bashrc
```

### Assets Prontos:
- **URDF G1 5-dedos**: `unitree_ros/robots/g1_description/g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf`
- **Meshes completos**: `unitree_ros/robots/g1_description/meshes/`
- **Ambiente 3D**: `3d escritorio/body1_structure/` (paredes prontas)

## 📋 **TAREFAS PRIORITÁRIAS (TODO):**

### Fase 1 - Setup Gazebo:
- [ ] Verificar instalação do Gazebo Garden/Fortress
- [ ] Criar workspace ROS2 para simulação Gazebo
- [ ] Adaptar URDF G1 para compatibilidade Gazebo
- [ ] Teste básico: spawn G1 no mundo vazio

### Fase 2 - Ambiente 3D:
- [ ] Converter paredes OBJ → formato SDF/World Gazebo
- [ ] Integrar ambiente escritório no Gazebo world
- [ ] Validar escala ambiente (33m x 28m x 3.15m)
- [ ] Spawn G1 no ambiente completo

### Fase 3 - Controle ROS2:
- [ ] Implementar ros2_control para G1
- [ ] Controllers para locomoção bípede
- [ ] Controllers para mãos de 5 dedos
- [ ] Interface teleoperação

## 🎯 **METAS TÉCNICAS:**

### Hardware Alvo:
- **Robô**: Unitree G1Edu U6 V2 (29 DOF)
- **Mãos**: RH56DFQ-2R/2L (5 dedos, 17 sensores táteis/mão)
- **Altura**: 1.2m escala humana
- **Ambiente**: Escritório 33x28x3.15m (LiDAR scan)

### Performance Esperada:
- **Simulação realtime** no Gazebo
- **Controle ROS2** responsivo (<10ms latência)
- **Física estável** para locomoção bípede
- **Manipulação precisa** com mãos 5-dedos

## 📚 **EXPERIÊNCIA ACUMULADA:**

### Sucessos da Fase MuJoCo:
1. ✅ **Escala correta** validada (robô 1.2m vs paredes 3.15m)
2. ✅ **Conversão 3D** OBJ → XML funcional
3. ✅ **Integração robô+ambiente** sem conflitos
4. ✅ **Física realista** MuJoCo validada
5. ✅ **Mãos 5-dedos** funcionais no URDF

### Lições Aprendidas:
- Escala é crítica: usar fator 0.01 para conversão mm→m
- Paredes reais estão no grupo "Body1" do OBJ scan
- URDF G1 com 5-dedos já disponível e funcional
- PATH configuração necessária para comandos `simulate`

---

## 🚧 **COMANDOS ATUAIS - ESTRATÉGIA HÍBRIDA:**

### Verificação Setup (VALIDADO):
```bash
# ROS2 Jazzy - ✅ Funcional
echo $ROS_DISTRO  # jazzy

# Gazebo Harmonic - ✅ Instalado
gz sim --version  # 8.9.0

# Repositórios - ✅ Ambos disponíveis
ls -la unitree_ros/robots/g1_description/    # FONTE URDF
ls -la unitree_ros2/example/src/src/g1/      # INTERFACE REAL
```

### Próximos Comandos (FASE 1):
```bash
# Criar workspace ROS2 dedicado
mkdir -p ~/ros2_g1_ws/src && cd ~/ros2_g1_ws/src

# Copiar URDF base (unitree_ros ROS1 → ROS2)
cp -r ~/Workspaces/G1/unitree_ros/robots/g1_description .

# Link interface real (unitree_ros2)
ln -s ~/Workspaces/G1/unitree_ros2 .

# Instalar dependências Gazebo ROS2
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-gz-ros2-control -y
```

### Referência MuJoCo (AINDA FUNCIONAL):
```bash
# Baseline funcional para comparação
cd ~/Workspaces/G1/unitree_ros/robots/g1_description
simulate g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf
```

## Important Notes - RESEARCH-BASED

- **MuJoCo Phase**: ✅ COMPLETED and archived in `old_tests/` - **FUNCTIONAL BASELINE**
- **Current Phase**: 🚧 Gazebo ROS2 hybrid approach (unitree_ros + unitree_ros2)
- **Key Discovery**: 🔬 **Combine repositories** - URDF from ros1 + control from ros2
- **Critical Limitation**: ⚠️ Gazebo lacks native G1 walking controller
- **Solution Strategy**: Focus on perception/planning/manipulation + custom balance
- **Environment**: 3D office ready (scale/positioning validated in MuJoCo)
- **Next Action**: Port URDF unitree_ros → ROS2 + gz_ros2_control integration