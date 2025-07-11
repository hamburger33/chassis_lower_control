# framework说明
当前项目为本人在原有的比赛代码框架上二次修改的结果，本人依据此框架在我自己的底盘机器人上进行了二次开发，并对代码进行了一定的优化。由于是经过好几手后才到我手中，如果有任何版权问题可私信告知。
此代码为导航小车的下位机程序，通过UART通信与上位机进行双向通信，讲电机编码器信息传输给上位机计算里程计，并将上位机的控制信息转发至电调控制电机运动。

## BSP层(Board Sopport Package)
- 主要功能：实现映射功能。
  - 在本框架中，BSP层与cube高度耦合，对该层的修改往往需要使用cube重新生成工程。该层也是唯一允许直接出现HAL库函数的代码层，**在非BSP层编写代码时，如需使用HAL_...函数，请思考是否有同功能的BSP_...函数**。
  - 最简单的(如gpio)仅是对HAL库函数的封装。较为复杂的则会进行一定程度的处理(如can)

- 补充与修改：某款主控对应的BSP层应保持相同，当认为该层可能缺少部分功能或有错误时，请联系组长确认后解决并更新整个框架，**请勿自行修改**。

- 代码移植：BSP层也是在不同系列、型号的stm32间执行代码移植时主要需要关注的代码层。向功能更强系列移植一般只需要重配cube并重新组织BSP层的映射关系，而向功能较少的系列移植还需要去掉其不支持的功能，建议做到“刀法精湛”。一般不需要对其他两层进行修改。

- 子文件与文件夹：
  - bsp.c/h：该层核心文件，其中.h被include至main.c中，以实现整个代码层的初始化。include了该层所有模块的.h并调用各模块的初始化函数
  - interface文件夹：存放该层的外设相关部分。每一个成对的.c/h对应一种外设，当上面两个代码层需要使用某个外设时，这里的文件就是对应的交互接口。
  - lib文件夹：存放BSP层文件中**功能上**偏软件的部分。这些功能仍是基于单片机的硬件实现的，但是不对应具体外设，而更类似于库文件，在各种地方被需要，被include并使用其功能。例如包含bsp_log可以使用和printf相同用法的printf_log，将内容打印到ozone的窗口以辅助调试。

- 注册回调函数与接收：bsp-interface中，通信类外设模块有的定义了回调函数类型(函数指针类型)，可以将普通函数注册成回调函数。
  - 注册方法：注册也通过一个函数实现，这个函数叫回调函数注册函数，在bsp....h中声明
  - 注册条件：被注册的普通函数满足规定的回调函数格式(形参和返回值类型)，其所在文件包含了回调函数注册函数所在的bsp....h
  - 注册作用：一个普通函数被注册成为某个回调函数后，函数在接收到数据后或其他设定位置会被调用。

- 其他特点：是三层中唯一不是按照面向对象思想构建的一层

## HAL层(硬件抽象层，Hardware Abstraction layer)
- 主要功能：实现对设备的封装
- 子文件与子文件夹
  - hal.c/h:该层核心文件。会对该层所有模块初始化，调用各个模块的init函数，并封装成hal层的init函数放到main.c中执行
  - pub_sub.c/h:整个框架高度模块化，各个模块保持高度独立，publish与subscribe机制是APP层各个模块进行信息交互的重要机制
  - driver文件夹:该文件夹的每一个子文件夹对应一种设备，主要由一个对应的.c和.h文件构成
  - monitor文件夹:实现看门狗功能。提供回调函数和count可选
  - lib文件夹:该层软件库存放位置，这些功能与硬件无关，而是提供通用的数据结构和“算子”以供该层的其他部分调用

- driver要点：
  - 结构体：
    - config：在APP层进行driver的实例化时，需要传入一个对应类型的config结构体。该结构体的作用是传入driver初始化需要的参数，进行一定的设置，通过config的不同，同种类型的不同个体具有一些不同的性质。
  - 函数：.c中存放的相当于这个类的private函数，.h中的则相当于public。相似的driver的public函数应较为统一。由于通信格式，使用方法等的不同，不同通信设备在读取操作、数据格式上可能有所不同，这些不同应该在driver的内部处理。
    - create函数：类似构造函数，创建一个该类型的driver。
    - init函数：对这一整个driver类进行初始化操作，创建序列等，保证能正常create与使用该driver类型。
  - 封装程度：应尽可能使到上层使用时不考虑下层所需的操作。如在使用电机时，这个电机的数据该和哪些电机的数据在一个数据包中发送，can的过滤器设置，均属于应该自动处理的功能；
  接收类的driver应该封装到只有init、create两个函数和一个实时更新的数据结构体

HAL层主要存放的是类，但在该层没有进行实例化，若在APP层没有实例化，则该模块的存在与否基本不会影响编译后的可执行文件，只会占用cvector初始化所需的少量内存。因此，基于本框架的工程没有必要删除APP层未使用的模块。


## APP层(application)
- 主要功能：实现机器人的控制

在完成BSP层与HAL层后，如果在APP层没有控制代码，则代码并无实际功能。换言之，BSP层与APP层的存在是为了APP层更简单、更合理、更易于扩展和移植。本框架的初始目标即是实现：在APP层仅需思考逻辑并用无关硬件的C语言代码实现即可完成整个机器人的控制。

- APP层按照模块（如云台、发射、底盘）可以建立数个子文件夹。采用了和HAL层类似的create机制，但目前没有传入config,若后期增加config可以实现与HAL层类似的一项功能：挂载多个同类型模块(如双枪管步兵create两个shoot模块即可，将不同的电机号写进config参数)。每个模块内的逻辑代码与原框架下的代码大同小异。 

- pub-sub机制及其使用：
  - 基本说明：该机制在本框架中用于APP层各模块之间的信息传递，能同时保证两个模块间是松散耦合。一个话题(topic)有一个发布者(publisher)和一定数量的订阅者(subscriber)，发布者在一个话题下发布(publish)变更后，所有订阅(subscribe)该话题的订阅者都会收到该变更。是否有订阅者、订阅者数量不会影响发布者发布变化。
  - topic：话题。形式是一个字符串，同一话题的发布者和订阅者使用相同字符串即可，不需要用变量将该字符串存储起来。因此该字符串最好规范并有明确含义。
  - Publisher/Subscriber：driver。和其他driver使用方法相同，在模块(如chassis,cmd)的结构体中写指针，在模块的Create里初始化。但由于pub-sub并不对应实体，初始化时不使用create而使用更类似底层的register函数，传入参数为topic。register_sub函数还有一个参数(buff_len)，该参数为队列最大长度，由于队列先进先出特性，队列中最多存储发布者最后buff_len次发布的消息，sub会得到该队列中最先收到（进入队列）的一条（即若每次读取会得到对应发布者倒数第1~buff_len次发布的消息），目前用到的都是即发即收，只需接收最新的消息，因此该参数给1即可。
  - 格式：pubsub是一种类似模块间通信的机制，所以需要发布者和订阅者同样的数据格式和长度。下面“发布过程”“订阅接收过程”中的“结构体A”即是数据格式，两边使用同一个A即可，而A的内容并不影响，因为pubsub机制只是会原封不动的收发数据。而数据长度就是sizeof(struct A)。建议在两个模块都会包含的头文件中定义A的形式。而pubsub机制并不会关注每一个topic下的“结构体A”的具体格式和内容，只按照给的指针和长度发送，相当于发送一个uint8_t数组。
  - 发布过程：
    1. 准备要发送的数据结构体。publish出的内容实质上是传出一个指针，该指针要能够被subscriber能读取到，因此它不能是在发送位置函数内创建的局部指针。推荐在模块的结构体中写Publisher的时同时创建一个对应的数据结构体A，然后用它的指针。A中包含需要传递的所有变量数据，在每次发送前将所有A中的变量赋好值。
    2. 将A的指针转化成能发出的格式并打包发出。创建一个publish_data类型的变量（随用随创一个局部变量即可），将1.中的结构体A的指针/结构体A的大小赋值给publish_data.data/.len(指针强转类型至uint8_t*避免warning)，并调用Publisher的“成员函数”publish以发布变化即可
  - 订阅接收过程
    1. 创建一个publish_data类变量用以接收publish出来的同类型变量，并调用Subscriber的“成员函数”getdata进行接收。
    2. 判断是否接收成功。上述publish_data型变量的.len长度为-1则说明未接收到数据。
    3. 使用数据。将publish_data型变量的.data转成结构体A的指针A*格式即可得到数据并使用

- #ifdef-#endif实现两块同一机器人不同主控使用同一个工程
  - 理论基础：两份工程的不同部分较少且集中
    - 由前面的说明易知，本框架下相同主控的不同工程中BSP和HAL层没有区别。 因此同一个机器人两块主控相同时，两份工程的区别集中于APP层。
    - 发射、底盘、云台、救援等模块均为类似HAL层的类-指针-实例化模式，若底盘主控代码里有云台的文件夹和代码，只要不在app.c创建指针并实例化就并不会实际生效，即在底盘主控中并不需要特意删除APP层的gimbal文件夹和文件。
    - robot_param.h中为定义和格式，并不影响
    - 综上，最终两份工程不同的代码集中在app.c和robot_cmd模块，因此具有代码复用的可行性。
  - 实现方式：robot_param.h中有等同于主控数量的宏定义，编译下载时注释到只剩下需要的一个；代码中对于多个主控不同的部分前面加#ifdef 对应主控宏定义，后面加#endif
  - 优点：
    - 一份工程维护起来更加容易，例如在对BSP和HAL进行统一更新时少一半工作量，占用开发者电脑空间、git服务器空间都会更小
    - 易于开发。如编写双板通信时不需多余操作即可保证两边的通信配置、数据包格式完全相同。
  - 缺点：
    - 需要在每次下载前确认宏定义是否正确，若切换宏定义必须重新编译(经分析目前如果搞错会因为各种重要模块丢失等进入stop模式，因此没有想办法处理，实际使用中仍需注意)
    - 需要一个整个app层都能include的头文件存放宏定义(robot_param.h)
