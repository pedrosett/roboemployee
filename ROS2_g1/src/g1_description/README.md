# G1 Description Package - Documentação Técnica

## 📋 Status Atual do Projeto

**Data**: 05/08/2024  
**Fase**: FASE 1 - Setup Básico G1 no Gazebo Vazio  
**Progresso**: FASE 1.4 Concluída, iniciando FASE 1.5 (Compilação e Teste)

### ✅ Fases Concluídas
- [x] **FASE 1.1**: Pacotes ROS-Gazebo instalados
- [x] **FASE 1.2**: Workspace ROS2 criado (`~/Workspaces/G1/ROS2_g1/`)
- [x] **FASE 1.3**: URDF G1 copiado do `unitree_ros`
- [x] **FASE 1.4**: Arquivos ROS2 criados (package.xml, CMakeLists.txt, configs)

### 🚧 Fase Atual
- [ ] **FASE 1.5**: Compilação workspace e teste G1 no Gazebo vazio

## 🤖 Análise dos Modelos Unitree G1 Disponíveis

### 📊 **Comparação Detalhada dos Arquivos**

| Arquivo | DOF | Mãos | Cintura | Formato | Uso Recomendado |
|---------|-----|------|---------|---------|-----------------|
| `g1_23dof.urdf/xml` | 23 | ❌ Sem mãos | ✅ Livre | URDF/XML | 🔧 Desenvolvimento locomoção |
| `g1_29dof.urdf/xml` | 29 | ❌ Sem mãos | ✅ Livre | URDF/XML | 🔧 Desenvolvimento braços |
| `g1_29dof_lock_waist.urdf/xml` | 29 | ❌ Sem mãos | 🔒 Bloqueada | URDF/XML | 🔧 Estabilidade inicial |
| `g1_29dof_rev_1_0.urdf/xml` | 29 | ❌ Sem mãos | ✅ Livre | URDF/XML | 🔧 Versão oficial |
| `g1_29dof_lock_waist_rev_1_0.urdf/xml` | 29 | ❌ Sem mãos | 🔒 Bloqueada | URDF/XML | 🔧 Estabilidade v1.0 |
| **`g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf`** | **43** | **✅ 5-dedos DFQ** | **✅ Livre** | **URDF** | **🎯 NOSSO ALVO** |

### 🎯 **Modelo Selecionado: G1 29DOF + Inspire Hand DFQ**

**Arquivo**: `g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf`  
**Especificações**:
- **DOF Total**: 43 (29 corpo + 14 mãos)
- **Mãos**: RH56DFQ-2R/2L (5 dedos por mão)
- **Cintura**: Livre (não bloqueada)
- **Versão**: rev_1_0 (versão oficial)

**Justificativa da Escolha**:
1. ✅ **Mãos 5-dedos** - Capacidade de manipulação avançada
2. ✅ **DOF completo** - Máxima flexibilidade
3. ✅ **Versão oficial** - Testada e validada
4. ✅ **Compatível** - Funciona tanto em MuJoCo quanto adaptável ao Gazebo

## 📁 Estrutura de Formatos

### **URDF vs XML - Diferenças**

| Aspecto | URDF (.urdf) | XML MuJoCo (.xml) |
|---------|--------------|-------------------|
| **Propósito** | ROS/Gazebo | MuJoCo nativo |
| **Sintaxe** | XML padronizado ROS | XML MuJoCo específico |
| **Compatibilidade** | ROS1/ROS2, Gazebo | MuJoCo exclusivo |
| **Física** | Básica | Avançada (contatos, elasticidade) |
| **Uso no projeto** | 🎯 Gazebo ROS2 | ✅ MuJoCo (já funcional) |

### **Numeração DOF Explicada**

| Modelo | DOF | Detalhamento |
|--------|-----|--------------|
| **23 DOF** | 23 | 12 pernas + 6 braços + 5 torso/cabeça |
| **29 DOF** | 29 | 23 DOF + 6 punhos/ombros adicionais |
| **43 DOF** | 43 | 29 DOF + 14 mãos (7 por mão × 2) |

## 🎮 **Estratégia Dual: MuJoCo + Gazebo**

### **MuJoCo (Já Funcional)**
```bash
# Comando testado e validado
cd ~/Workspaces/G1/unitree_ros/robots/g1_description
simulate g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf
```

### **Gazebo ROS2 (Em Desenvolvimento)**
```bash
# Objetivo final (em implementação)
cd ~/Workspaces/G1/ROS2_g1
colcon build --symlink-install
source install/setup.bash
ros2 launch g1_description g1_empty_world.launch.py
```

## 🛠️ **Configuração Atual do Workspace**

### **Estrutura ROS2**
```
~/Workspaces/G1/ROS2_g1/
├── .vscode/tasks.json          # Build configuration
└── src/g1_description/
    ├── package.xml             # Dependências ROS2
    ├── CMakeLists.txt          # Build system
    ├── config/
    │   └── g1_controllers.yaml # ros2_control config
    ├── urdf/
    │   └── g1_gazebo.urdf      # URDF adaptado Gazebo
    ├── launch/
    │   └── g1_empty_world.launch.py # Launch Gazebo
    ├── meshes/                 # STL files (copiados)
    └── g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf # URDF base
```

### **Dependências Instaladas**
- ✅ `ros-jazzy-ros-gz-sim`
- ✅ `ros-jazzy-ros-gz-bridge`
- ✅ `ros-jazzy-gz-ros2-control`
- ✅ `ros-jazzy-ros2-control`
- ✅ `ros-jazzy-ros2-controllers`
- ✅ `ros-jazzy-xacro`

## 🔄 **Adaptações URDF → Gazebo**

### **Plugins Adicionados**
```xml
<!-- Plugin gz_ros2_control adicionado -->
<gazebo>
  <plugin name="gz_ros2_control" filename="libgz_ros2_control.so">
    <parameters>$(find g1_description)/config/g1_controllers.yaml</parameters>
  </plugin>
</gazebo>

<!-- ros2_control interfaces configuradas -->
<ros2_control name="g1_system" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSystem</plugin>
  </hardware>
  <!-- 29 joints corpo + 14 joints mãos configurados -->
</ros2_control>
```

### **Controladores Configurados**
- ✅ `g1_joint_state_broadcaster` - Estados das juntas
- ✅ `g1_hand_position_controller` - Controle posição mãos
- ✅ `g1_leg_effort_controller` - Controle esforço pernas

## ⚠️ **Limitações Conhecidas (Baseadas na Pesquisa)**

### **Gazebo vs MuJoCo**
| Aspecto | MuJoCo | Gazebo |
|---------|---------|---------|
| **Locomoção nativa** | ✅ Funcional | ❌ Precisa implementar |
| **Física avançada** | ✅ Superior | ⚠️ Básica |
| **ROS2 integração** | ⚠️ Via bridge | ✅ Nativa |
| **Sensores** | ✅ Completos | ✅ Completos |

### **Próximos Desafios**
1. **Estabilização**: G1 precisa ficar em pé (Gazebo não tem controle nativo)
2. **Locomoção**: Implementar algoritmos próprios de caminhada
3. **Sim-to-Real**: Interface consistente simulação ↔ hardware real

## 🎯 **Próximos Passos**

### **FASE 1.5 - Compilação e Teste** (Em Andamento)
1. [ ] **Build workspace**: `colcon build --symlink-install`
2. [ ] **Source environment**: `source install/setup.bash`
3. [ ] **Test spawn**: `ros2 launch g1_description g1_empty_world.launch.py`
4. [ ] **Verify joints**: `ros2 topic echo /joint_states`

### **CHECKPOINT 1 - Critérios de Sucesso**
- ✅ G1 aparece no Gazebo
- ✅ Mãos 5-dedos visíveis
- ✅ 43 joints funcionais
- ✅ Não cai imediatamente
- ✅ `/joint_states` publicando

## 📚 **Comandos de Referência**

### **Build e Test**
```bash
# Build (via VSCode ou manual)
cd ~/Workspaces/G1/ROS2_g1
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Test Gazebo
source install/setup.bash
ros2 launch g1_description g1_empty_world.launch.py

# Test MuJoCo (alternativo)
cd ~/Workspaces/G1/unitree_ros/robots/g1_description
simulate g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf
```

### **Debug**
```bash
# Verificar tópicos
ros2 topic list | grep joint

# Verificar controladores
ros2 control list_controllers

# Verificar meshes
find meshes/ -name "*.STL" | grep hand
```

---

**Status**: 🚧 **DESENVOLVIMENTO ATIVO**  
**Próximo**: Build + Spawn test no Gazebo  
**Fallback**: MuJoCo simulação já validada e funcional
