# Unitree G1 Humanoid Robot - Gazebo ROS2 Simulation

Projeto de simulação do robô humanoide **Unitree G1** com mãos de 5 dedos no **Gazebo** integrado ao **ROS2 Jazzy**, incluindo ambiente 3D real escaneado via LiDAR.

## 🎯 Objetivos da Nova Fase

### Migração MuJoCo → Gazebo ROS2
- **Fase Anterior (CONCLUÍDA)**: Simulação MuJoCo funcional arquivada em [`old_tests/`](old_tests/)
- **Fase Atual**: Desenvolvimento no **Gazebo ROS2** para maior integração com ROS ecosystem

### 🔬 **Abordagem Técnica Baseada em Pesquisa**

**Estratégia Híbrida: Combinação unitree_ros + unitree_ros2**
- **unitree_ros (ROS1)**: Fonte dos modelos URDF/meshes completos do G1 (29 DOF + mãos Dex5/Inspire)
- **unitree_ros2 (ROS2)**: Interface nativa para comunicação DDS com robôs reais via ROS2
- **Abordagem**: Portar URDF do unitree_ros para ambiente ROS2 + usar unitree_ros2 para controle real

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
- **Simulador**: Gazebo Harmonic (gz sim v8.9.0) - Ignition/Garden
- **Linguagens**: Python 3.12, C++17
- **Integração**: `ros_gz_sim`, `gz_ros2_control`, CycloneDDS

### Hardware Alvo
- **Robô**: Unitree G1Edu U6 V2
- **Mãos**: RH56DFQ-2R/2L (5 dedos, 17 sensores táteis por mão) - Inspire Hand  
- **Altura**: 1.2m (escala humana)
- **SDK**: Unitree SDK2 para comunicação DDS nativa

### 📊 **Repositórios e Suas Funções**

| Repositório | Função Principal | Uso no Projeto |
|-------------|------------------|----------------|
| **unitree_ros** | Modelos URDF/meshes (ROS1) | 🎯 **Fonte dos modelos G1** para simulação |
| **unitree_ros2** | Comunicação DDS (ROS2) | 🎯 **Controle do robô real** via ROS2 |
| **Combinação** | Simulação + Real | ✅ **Estratégia adotada** para consistência |

## 📁 Estrutura do Projeto

```
~/Workspaces/G1/
├── README.md                     # Este arquivo
├── CLAUDE.md                     # Instruções para IA
├── old_tests/                    # 📦 Simulações MuJoCo arquivadas (FUNCIONAIS)
├── unitree_ros/                  # 🎯 **FONTE URDF** - Modelos G1 completos (ROS1)
│   └── robots/g1_description/    # G1 29DOF + mãos Inspire 5-dedos (TESTADO)
├── unitree_ros2/                 # 🎯 **CONTROLE REAL** - Interface ROS2 nativa
│   ├── cyclonedx_ws/            # Mensagens DDS para comunicação
│   └── example/                 # Exemplos G1, Go2, H1 em ROS2
├── 3d_escritorio/                # 🏢 Assets 3D do ambiente LiDAR
│   ├── escritorio_CW_scan.obj    # Scan original (131 grupos)
│   └── body1_structure/          # Paredes extraídas (33x28x3.15m)
├── unitree_sdk2/                 # SDK C++ oficial Unitree
├── unitree_sdk2_python/          # Interface Python SDK2
├── gazebo_workspace/             # 🔧 Workspace ROS2 para Gazebo (a criar)
└── docs/                         # 📚 Documentação técnica
```

## 🚀 Roadmap de Desenvolvimento

### **Metodologia: Abordagem Híbrida Validada por Pesquisa**

### Fase 1: Porte URDF unitree_ros → ROS2 Gazebo
- [ ] **Extrair modelo G1**: Copiar URDF/meshes do `unitree_ros/robots/g1_description/`
- [ ] **Adaptar para Gazebo Harmonic**: Substituir plugins ROS1 → `gz_ros2_control`
- [ ] **Configurar ros2_control**: Controladores para juntas (posição/torque/velocidade)
- [ ] **Teste básico**: Spawn G1 no Gazebo vazio com mãos funcionais

### Fase 2: Integração Ambiente 3D Escritório
- [ ] **Converter OBJ → SDF**: Body1_structure.obj → modelo Gazebo estático
- [ ] **Criar office.world**: Incluir paredes + ground_plane + iluminação
- [ ] **Validar escala**: Robô 1.2m vs paredes 3.15m (experiência MuJoCo)
- [ ] **Teste integrado**: G1 no ambiente escritório completo

### Fase 3: Controle ROS2 Unificado
- [ ] **Simulação**: Controladores `gz_ros2_control` para juntas individuais
- [ ] **Hardware**: Integrar `unitree_ros2` para comandos de alto nível
- [ ] **Interface consistente**: Tópicos compatíveis sim ↔ real
- [ ] **Sensores**: IMU, joint_states, câmeras funcionais

### Fase 4: Limitações e Otimizações
- [ ] **Estabilização**: Implementar controle de equilíbrio (Gazebo não inclui)
- [ ] **Locomoção**: Algoritmos próprios para caminhada bípede
- [ ] **Manipulação**: Controle preciso mãos 5-dedos Inspire
- [ ] **Sim-to-Real**: Transferência consistente simulação → hardware

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
- **G1 Specs**: 29 DOF, 1.2m altura, mãos RH56DFQ (Inspire Hand/Dex5)
- **Ambiente**: Escritório 33m x 28m x 3.15m (LiDAR scan)
- **Experiência Prévia**: Simulação MuJoCo 100% funcional (ver `old_tests/`)

### 📚 **Pesquisa Técnica - Repositórios Unitree**

**Diferenças Fundamentais:**
- **unitree_ros**: ROS1, foco em **simulação** (URDF/Gazebo/meshes completos)
- **unitree_ros2**: ROS2, foco em **controle real** (DDS/SDK2, sem simulação)
- **Solução**: **Combinar ambos** - URDF do ros1 + controle ros2

**Limitações Importantes:**
- ⚠️ **Gazebo não inclui controlador de caminhada nativo do G1**
- ⚠️ **Controle simulação ≠ controle real** (baixo nível vs alto nível)
- ✅ **Solução**: Desenvolver algoritmos próprios de equilíbrio/locomoção

**Gazebo: Clássico vs Ignition/Harmonic:**
- ✅ **Escolhido**: Gazebo Harmonic (v8.9.0) com `ros_gz_sim`
- **Vantagem**: Melhor física, rendering, integração ROS2 nativa
- **Trade-off**: Mais complexo que clássico, mas futuro-prova

## ⚠️ Status Atual

**🔄 INICIANDO NOVA FASE**
- ✅ **MuJoCo Phase**: Concluída com sucesso (arquivada)
- 🚧 **Gazebo ROS2 Phase**: Iniciando setup e desenvolvimento
- 📋 **Next Action**: Configurar workspace Gazebo ROS2

---

*Última atualização: $(date '+%d/%m/%Y %H:%M')*  
*Versão anterior (MuJoCo): Disponível em [`old_tests/`](old_tests/)*