# DebugSerial
mbedのシリアルクラスにエスケープシーケンスを付加したもの

## 使用方法

#### インスタンス生成
Serialクラスを継承しているので基本的な使い方は同じ
```cpp
DebugSerial pc(USBTX, USBRX);
```
```cpp
DebugSerial pc(USBTX, USBRX, baudrate);
```

#### clsコマンド再現
```cpp
pc.cls();
```

#### 全ての状態をリセット
```cpp
pc.reset();
```

#### 画面クリア
```cpp
pc.clear();
```

#### カーソル位置指定
```cpp
pc.setPos(x, y);
```

#### カーソル移動
```cpp
pc.moveCursor(distance, dirction);
```
```cpp
// dirction
enum cursorDir
{
    cursor_up,
    cursor_down,
    cursor_right,
    cursor_left
};
```

または

```cpp
pc.up(distance);
pc.down(distance);
pc.right(distance);
pc.left(distance);
```

#### カーソル位置から行末までをクリア
```cpp
pc.clearToLineEnd();
```

#### 文字属性・デフォルト
```cpp
pc.setLetterDefault();
```

#### 文字属性・協調
```cpp
pc.setBrightBold(bool isOn);
```

#### 文字属性・下線
```cpp
pc.setUnderLine(bool isOn);
```

#### 文字属性・反転
```cpp
pc.setFlip(bool isOn);
```

#### 文字色変更
```cpp
pc.setLetterColor(c);
```
```cpp
// c
enum color
{
    color_black,
    color_red,
    color_green,
    color_yellow,
    color_blue,
    color_magenta,
    color_cyan,
    color_gray,
    color_default
};
```
#### 背景色変更
```cpp
pc.setBackColor(c);
```
```cpp
// c
enum color
{
    color_black,
    color_red,
    color_green,
    color_yellow,
    color_blue,
    color_magenta,
    color_cyan,
    color_gray,
    color_default
};
```
