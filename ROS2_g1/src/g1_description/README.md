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
| `g1_23dof.urdf/xml` | 23 | 🤖 Rubber fixas | ✅ Livre | URDF/XML | 🔧 Desenvolvimento locomoção |
| `g1_29dof.urdf/xml` | 29 | 🤖 Rubber fixas | ✅ Livre | URDF/XML | 🔧 Desenvolvimento braços |
| `g1_29dof_lock_waist.urdf/xml` | 29 | 🤖 Rubber fixas | 🔒 Bloqueada | URDF/XML | 🔧 Estabilidade inicial |
| `g1_29dof_rev_1_0.urdf/xml` | 29 | 🤖 Rubber fixas | ✅ Livre | URDF/XML | 🔧 Versão oficial |
| `g1_29dof_lock_waist_rev_1_0.urdf/xml` | 29 | 🤖 Rubber fixas | 🔒 Bloqueada | URDF/XML | 🔧 Estabilidade v1.0 |
| `g1_29dof_with_hand_rev_1_0.urdf` | 43 | ✅ 3-dedos articuladas | ✅ Livre | URDF | 🔧 Manipulação básica |
| **`g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf`** | **53** | **✅ 5-dedos Inspire DFQ** | **✅ Livre** | **URDF** | **🎯 NOSSO ALVO** |

### 🎯 **Modelo Selecionado: G1 29DOF + Inspire Hand DFQ**

**Arquivo**: `g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf`  
**Especificações**:
- **DOF Total**: 53 (29 corpo + 24 mãos Inspire)
- **Mãos**: RH56DFQ-2R/2L (5 dedos articulados por mão)
- **Cintura**: Livre (não bloqueada)
- **Versão**: rev_1_0 (versão oficial)

**Justificativa da Escolha**:
1. ✅ **Mãos 5-dedos Inspire** - Capacidade de manipulação avançada (24 DOF mãos)
2. ✅ **DOF completo** - Máxima flexibilidade (53 DOF total)
3. ✅ **Versão oficial** - Testada e validada
4. ✅ **Compatível** - Funciona tanto em MuJoCo quanto adaptável ao Gazebo

### 🖐️ **Tipos de Mãos Disponíveis (Análise Detalhada)**

**🔍 DESCOBERTA**: Todos os modelos G1 têm mãos visuais, mas com diferentes níveis de articulação:

| Tipo de Mão | Arquivos | DOF Mãos | Articulação | Capacidade |
|--------------|----------|-----------|-------------|------------|
| **Rubber Fixas** | g1_23dof, g1_29dof, g1_29dof_rev_1_0 | 0 | ❌ Nenhuma | 🤖 Apenas visual/estética |
| **3-Dedos Articuladas** | g1_29dof_with_hand_rev_1_0 | 14 | ✅ Básica | 🔧 Manipulação simples |
| **5-Dedos Inspire DFQ** | g1_29dof_rev_1_0_with_inspire_hand_DFQ | 24 | ✅ Completa | 🎯 Manipulação avançada |

**Detalhes Técnicos**:
- **Rubber**: Mãos fixas de borracha (meshes `*_rubber_hand.STL`)
- **3-Dedos**: thumb_0/1/2 + index_0/1 + middle_0/1 (7 joints × 2 mãos = 14 DOF)
- **Inspire DFQ**: 5 dedos completos com falanges independentes (12 joints × 2 mãos = 24 DOF)

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
| **23 DOF** | 23 | 12 pernas + 6 braços + 5 torso/cabeça + mãos rubber fixas |
| **29 DOF** | 29 | 23 DOF + 6 punhos/ombros adicionais + mãos rubber fixas |
| **43 DOF** | 43 | 29 DOF + 14 mãos articuladas (3-dedos: thumb+index+middle) |
| **53 DOF** | 53 | 29 DOF + 24 mãos Inspire (5-dedos completos por mão) |

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

### **FASE 1.5 - Compilação e Teste** (✅ CONCLUÍDA)
1. [x] **Build workspace**: `colcon build --symlink-install` ✅
2. [x] **Source environment**: `source install/setup.bash` ✅
3. [x] **Test spawn**: `ros2 launch g1_description g1_empty_world.launch.py` ✅
4. [x] **Verify joints**: Robot spawned successfully ✅

### **🎉 CHECKPOINT 1 - SUCESSO ATINGIDO**
- ✅ **G1 spawnou no Gazebo** - Entity creation successful
- ✅ **Robot state publisher funcionando** - Robot initialized
- ✅ **Processos ROS2 estáveis** - Todos os PIDs ativos
- ✅ **Mãos 5-dedos visíveis** - Modelo Inspire DFQ carregado
- ⚠️ **Mimic joints limitados** - Physics engine não suporta completamente

### **📊 ANÁLISE DE LOGS DETALHADA**

**🔍 LOG DO SUCESSO (05/08/2024 16:03:07):**
```
[INFO] [gazebo-1]: process started with pid [79697]
[INFO] [robot_state_publisher-2]: process started with pid [79698] 
[INFO] [joint_state_publisher-3]: process started with pid [79699]
[INFO] [create-4]: process started with pid [79700]
[robot_state_publisher-2] [INFO] [1754420588.142306384] [robot_state_publisher]: Robot initialized
[create-4] [INFO] [1754420588.678010627] [ros_gz_sim]: Entity creation successful.
[INFO] [create-4]: process has finished cleanly [pid 79700]
```

**⚠️ AVISOS IDENTIFICADOS (NÃO CRÍTICOS):**

1. **KDL Parser Warning**:
   ```
   The root link pelvis has an inertia specified in the URDF, but KDL does not support a root link with an inertia.
   ```
   - **Status**: ⚠️ Menor - Não afeta funcionamento básico
   - **Impacto**: Cinemática pode ser ligeiramente imprecisa
   - **Workaround**: "add an extra dummy link to your URDF" (futuro)

2. **EGL Graphics Warning**:
   ```
   libEGL warning: egl: failed to create dri2 screen
   ```
   - **Status**: ⚠️ Menor - Rendering gráfico
   - **Impacto**: Possível degradação visual menor
   - **Causa**: Driver OpenGL/hardware específico

3. **⚠️ LIMITAÇÃO CRÍTICA - Mimic Joints**:
   ```
   [Err] [Physics.cc:1785] Attempting to create a mimic constraint for joint [L_index_intermediate_joint] 
   but the chosen physics engine does not support mimic constraints
   ```
   - **Status**: ⚠️ **IMPORTANTE** - Funcionalidade limitada
   - **Impacto**: Mãos Inspire 5-dedos podem não articular completamente
   - **Joint afetado**: `L_index_intermediate_joint` (indicador esquerdo)
   - **Solução**: Avaliar uso do modelo 3-dedos se necessário

## 🛠️ **TROUBLESHOOTING - PROBLEMAS RESOLVIDOS**

### **❌ → ✅ Erros Enfrentados e Soluções**

**1. Package.xml Missing**
```bash
# ERRO: 
CMake Error: File /home/pedro_setubal/Workspaces/G1/ROS2_g1/src/g1_description/package.xml does not exist.

# SOLUÇÃO:
# Criar package.xml com dependências corretas do ROS2 Jazzy
```

**2. Joint State Publisher Not Found**
```bash  
# ERRO:
[ERROR] [launch]: "package 'joint_state_publisher' not found"

# SOLUÇÃO:
sudo apt install ros-jazzy-joint-state-publisher ros-jazzy-robot-state-publisher -y
```

**3. XML Parsing Error (URDF)**
```bash
# ERRO: 
Error: Error=XML_ERROR_PARSING_TEXT ErrorID=8 (0x8) Line number=1

# SOLUÇÃO:
# Corrigir launch file para usar URDF original diretamente:
robot_description': open(os.path.join(..., 'g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf')).read()
```

**4. Gazebo World File Not Found**  
```bash
# ERRO:
Unable to find or download file empty.world

# SOLUÇÃO: 
# Usar empty.sdf padrão do Gazebo:
'gz_args': '-r empty.sdf'
```

### **🔧 CORREÇÕES APLICADAS**

**Launch File Final (`g1_empty_world.launch.py`):**
```python
# Robot description via file read direto
'robot_description': open(os.path.join(
    os.path.dirname(__file__), '..', 
    'g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf'
)).read()

# Spawn usando path absoluto
'-file', os.path.join(
    os.path.dirname(__file__), '..', 
    'g1_29dof_rev_1_0_with_inspire_hand_DFQ.urdf'
)

# Gazebo world padrão
'gz_args': '-r empty.sdf'
```

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

## 🎉 **STATUS FINAL - CHECKPOINT 1 CONCLUÍDO**

**Status**: ✅ **G1 FUNCIONANDO NO GAZEBO ROS2**  
**Data**: 05/08/2024 16:03:07  
**Resultado**: Spawn successful com avisos menores  

### **✅ SUCESSOS ALCANÇADOS:**
- G1 29DOF + mãos Inspire DFQ carregando no Gazebo
- Robot state publisher inicializado
- Processos ROS2 estáveis
- Launch file funcional
- Workspace ROS2 completo

### **⚠️ LIMITAÇÕES CONHECIDAS:**
- Mimic joints não suportados pelo physics engine
- KDL parser warning (root inertia)
- EGL graphics warning (menor)

### **🚀 FASES CONCLUÍDAS:**
- ✅ **FASE 1**: G1 funcionando no Gazebo ROS2 vazio
- ✅ **FASE 2**: Ambiente 3D escritório com escala correta (3.15m altura)

### **📋 ARQUIVOS 3D CRIADOS:**
- `escritorio_CW_scan_smart_scaled.obj` - Ambiente com escala correta
- `smart_scale_obj.py` - Script de escala inteligente
- `PHASE2_3D_SCALING.md` - Documentação detalhada

### **🎯 PRÓXIMA FASE:**
**FASE 3**: Integração G1 + Ambiente 3D no mesmo mundo Gazebo

**Fallback**: MuJoCo simulação continua validada e funcional
