# Unitree G1 Humanoid Robot - Gazebo ROS2 Simulation

Projeto de simulação do robô humanoide **Unitree G1** com mãos de 5 dedos no **Gazebo** integrado ao **ROS2 Jazzy**, incluindo ambiente 3D real escaneado via LiDAR.

## 🎯 Objetivos da Nova Fase

### Migração MuJoCo → Gazebo ROS2
- **Fase Anterior (CONCLUÍDA)**: Simulação MuJoCo funcional arquivada em [`old_tests/`](old_tests/)
- **Fase Atual**: Desenvolvimento no **Gazebo ROS2** para maior integração com ROS ecosystem

### Metas Técnicas
1. **Robô G1 Completo**: 29 DOF + mãos RH56DFQ de 5 dedos no Gazebo
2. **Ambiente 3D**: Integrar paredes do escritório (33m x 28m x 3.15m) escaneadas via LiDAR  
3. **ROS2 Integration**: Controle via tópicos, serviços e actions
4. **Sensores Realistas**: IMU, câmeras, sensores táteis
5. **Controle Avançado**: Algoritmos de locomoção e manipulação

## 🛠️ Stack Tecnológico

### Ambiente de Desenvolvimento
- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS2 Jazzy 
- **Simulador**: Gazebo Garden/Fortress
- **Linguagens**: Python 3.12, C++17

### Hardware Alvo
- **Robô**: Unitree G1Edu U6 V2
- **Mãos**: RH56DFQ-2R/2L (5 dedos, 17 sensores táteis por mão)
- **Altura**: 1.2m (escala humana)

## 📁 Estrutura do Projeto

```
~/Workspaces/G1/
├── README.md                     # Este arquivo
├── CLAUDE.md                     # Instruções para IA
├── old_tests/                    # 📦 Simulações MuJoCo arquivadas
├── unitree_ros/                  # 🤖 Pacotes ROS2 oficiais Unitree
│   └── robots/g1_description/    # URDF/meshes do G1 com 5 dedos
├── 3d_escritorio/                # 🏢 Assets 3D do ambiente
│   ├── escritorio_CW_scan.obj    # Scan LiDAR original (131 grupos)
│   └── body1_structure/          # Paredes extraídas (MuJoCo format)
├── gazebo_workspace/             # 🔧 Workspace ROS2 para Gazebo (a criar)
└── docs/                         # 📚 Documentação técnica
```

## 🚀 Roadmap de Desenvolvimento

### Fase 1: Setup Base Gazebo
- [ ] Configurar workspace ROS2 para Gazebo
- [ ] Converter/adaptar URDF do G1 para Gazebo
- [ ] Testar spawn do robô no Gazebo vazio
- [ ] Validar mãos de 5 dedos funcionais

### Fase 2: Integração do Ambiente 3D  
- [ ] Converter paredes OBJ para formato Gazebo (SDF/World)
- [ ] Integrar ambiente de escritório no Gazebo world
- [ ] Validar escala e física do ambiente
- [ ] Spawn G1 no ambiente integrado

### Fase 3: Controle ROS2
- [ ] Implementar controllers ROS2 para G1
- [ ] Interface de controle via tópicos
- [ ] Sensores (IMU, câmeras, tátil) funcionais
- [ ] Teleoperação básica

### Fase 4: Algoritmos Avançados
- [ ] Controle de locomoção bípede
- [ ] Manipulação com mãos de 5 dedos  
- [ ] Navegação no ambiente
- [ ] Comportamentos autônomos

## 📋 Pré-requisitos

### ROS2 Jazzy (✅ Instalado)
```bash
source /opt/ros/jazzy/setup.bash
```

### Gazebo (a verificar)
```bash
# Verificar instalação
gazebo --version
```

### Dependências Python
```bash
# Em ambiente virtual
pip install numpy opencv-python matplotlib
```

## 🏃‍♂️ Quick Start

### 1. Verificar Setup Atual
```bash
# Verificar ROS2
ros2 --version

# Verificar Gazebo  
gazebo --version

# Verificar G1 URDF
ls -la unitree_ros/robots/g1_description/
```

### 2. Primeiro Teste (quando implementado)
```bash
# Lançar Gazebo com G1
ros2 launch g1_gazebo g1_empty_world.launch.py

# Em outro terminal - controle básico
ros2 topic list
```

## 📖 Recursos de Aprendizado

### Documentação Oficial
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Garden Guide](https://gazebosim.org/docs/garden)
- [Unitree G1 Manual](old_tests/) (na documentação arquivada)

### Referências Técnicas
- **G1 Specs**: 29 DOF, 1.2m altura, mãos RH56DFQ
- **Ambiente**: Escritório 33m x 28m x 3.15m (LiDAR scan)
- **Experiência Prévia**: Simulação MuJoCo 100% funcional (ver `old_tests/`)

## ⚠️ Status Atual

**🔄 INICIANDO NOVA FASE**
- ✅ **MuJoCo Phase**: Concluída com sucesso (arquivada)
- 🚧 **Gazebo ROS2 Phase**: Iniciando setup e desenvolvimento
- 📋 **Next Action**: Configurar workspace Gazebo ROS2

---

*Última atualização: $(date '+%d/%m/%Y %H:%M')*  
*Versão anterior (MuJoCo): Disponível em [`old_tests/`](old_tests/)*