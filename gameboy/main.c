#include <stdio.h>
#include <stdlib.h>
typedef unsigned char byte;
typedef unsigned short word;
typedef enum {false, true} bool;
typedef enum {A, B, C, D, E, F, H, L, AF, BC, DE, HL, SP, PC} reg_name;
typedef enum {VBLANK, LCDC, TIMER, SERIAL, HILO} interupt_name;
typedef enum {NZ, NC, Z} cond_code;


typedef struct {
    word LCDC;
    word STAT;
    word SCY;
    word SCX;
    word LY;
    word LYC;
    word BGP;
    word OBP0;
    word OBP1;
    word WY;
    word WX;

    word OAM_START;
    word OAM_END;

    int HEIGHT;
    int WIDTH;
    word START_0;
    word START_1;
    word LENGTH; // 1024 bytes = 32*32

    int VBLANK_TIME;
    int clock;
    int mode;
    int line;

    byte buffer[23040];
    byte tileBuffer[8];
    byte bgTileCache[64][16];
    int usedCache[64];

}GPU;

typedef struct {
	int time;
	int div_time;
} Timer;

typedef struct {
	struct {
		int c;
		int serial;
	} clock;

	// Register set
	struct {
		byte a, b, c, d, e, h, l, f;    // 8-bit registers
		word pc, sp;                    // Program counter and stack pointer
		byte m, t;                      // Clock for last instruction
	} registers;

	// Saved register set
	struct {
		byte a, b, c, d, e, h, l, f;    // 8-bit registers
	} rsv;

	bool IME_flag;
	bool is_halted;
	bool is_paused;
	bool usingBootROM;
	Timer timer;
	GPU* gpu;
}CPU;

CPU* backup = NULL;

void requestInterrupt(interupt_name index);
// Memory Start
byte* base = (0x1900000 * 2);
//byte* ppu_base = 0x04000000;

byte readData(word address) {
	byte* val = base + address;
	return *val;
}

void resetDiv(Timer* timer);

void writeData(word address, byte data);
void dmaTransfer(GPU* gpu, byte startPrefix){
	printf("DMA Trans\n");
	word startAddress = startPrefix;
	startAddress = startAddress << 8;
	for(int i = 0; i < 0xA0; i++) {
		writeData(gpu->OAM_START + i, readData(startAddress + i));
	}
}

void writeData(word address, byte data) {

//	if(address < 0x8000 || (address >= 0xA000 && address < 0xC000)){
//
//	}else if(address >= 0xFF10 && address < 0xFF3F) {
//
//	}else if(address == 0xFF00) {
//		byte* val = base + address;
//		*val = (readData(address) & 0x0F) | (data & 0x30);
//	}else {
		byte* val = base + address;
		*val = data;

//		if((address & 0xFF00) == 0xFF00) {
//			if(address == 0xFF04){
//				resetDiv(&(backup->timer));
//			}
//			if(address == 0xFF46) {
//				dmaTransfer(backup->gpu, data);
//			}
//		}
//	}

}

void resetMemory() {
	word addr = 0x8000;
	while(addr != 0xA000) {
		writeData(addr, 0);
		addr += 1;
	}
	addr = 0xFF00;
	while(addr != 0xFF80) {
		writeData(addr, 0);
		addr += 1;
	}
	writeData(0xFFFF, 0);
}

// Memory End

//Start PPU
int* screenBase = 0x04000000;


void initGPU(GPU* gpu) {
    gpu->LCDC= 0xFF40;
    gpu->STAT= 0xFF41;
    gpu->SCY = 0xFF42;
    gpu->SCX = 0xFF43;
    gpu->LY  = 0xFF44;
    gpu->LYC = 0xFF45;
    gpu->BGP = 0xFF47;
    gpu->OBP0= 0xFF48;
    gpu->OBP1= 0xFF49;
    gpu->WY  = 0xFF4A;
    gpu->WX  = 0xFF4B;

    gpu->OAM_START = 0xFE00;
    gpu->OAM_END   = 0xFE9F;

    gpu->HEIGHT = 32;
    gpu->WIDTH = 32;
    gpu->START_0 = 0x9800;
    gpu->START_1 = 0x9C00;
    gpu->LENGTH = 0x0400;

    gpu->VBLANK_TIME = 70224;
    gpu->clock = 0;
    gpu->mode = 2;
    gpu->line = 0;

    //gpu.buffer = new Array(Screen.physics.WIDTH * Screen.physics.HEIGHT);
    //gpu.tileBuffer = new Array(8);
    //gpu.bgTileCache = {};
    for(int i = 0; i < 64; i++){
    	gpu->usedCache[i] = -1;
    }
}
void updateLY(GPU* gpu);
void setMode(GPU* gpu, int mode);
void drawFrame(GPU* gpu);
void drawScanLine(GPU* gpu, int line);
bool gpuUpdate(GPU* gpu, int clockElapsed) {
    gpu->clock += clockElapsed;
    bool vblank = false;

    switch (gpu->mode) {
        case 0: // HBLANK
            if (gpu->clock >= 204) {
                gpu->clock -= 204;
                gpu->line++;
                updateLY(gpu);
                if (gpu->line == 144) {
                    setMode(gpu, 1);
                    vblank = true;
                    requestInterrupt(VBLANK);
                    drawFrame(gpu);
                } else {
                    setMode(gpu, 2);
                }
            }
            break;
        case 1: // VBLANK
            if (gpu->clock >= 456) {
                gpu->clock -= 456;
                gpu->line++;
                if (gpu->line > 153) {
                    gpu->line = 0;
                    setMode(gpu, 2);
                }
                updateLY(gpu);
            }

            break;
        case 2: // SCANLINE OAM
            if (gpu->clock >= 80) {
                gpu->clock -= 80;
                setMode(gpu, 3);
            }
            break;
        case 3: // SCANLINE VRAM
            if (gpu->clock >= 172) {
                gpu->clock -= 172;
                drawScanLine(gpu, gpu->line);
                setMode(gpu, 0);
            }
            break;
    }

    return vblank;
}

void updateLY(GPU* gpu) {
    writeData(gpu->LY, gpu->line);
    byte STAT = readData(gpu->STAT);
    if (readData(gpu->LY) == readData(gpu->LYC)) {
        writeData(gpu->STAT, STAT | (1 << 2));
        if (STAT & (1 << 6)) {
//            requestInterrupt(LCDC);
        }
    } else {
        writeData(gpu->STAT, STAT & (0xFF - (1 << 2)));
    }
}

void setMode(GPU* gpu, int mode) {
    gpu->mode = mode;
    byte newSTAT = readData(gpu->STAT);
    newSTAT &= 0xFC;
    newSTAT |= mode;
    writeData(gpu->STAT, newSTAT);

    if (mode < 3) {
        if (newSTAT & (1 << (3+mode))) {
//            requestInterrupt(LCDC);
        }
    }
}
void drawBackground(GPU* gpu, byte LCDC, int line, byte* lineBuffer);
void drawSprites(GPU* gpu, byte LCDC, int line, byte* lineBuffer);
void drawScanLine(GPU* gpu, int line) {
    byte LCDC = readData(gpu->LCDC);
    byte enable = (LCDC >> 7) & 0x1;
    if (enable == 1) {
        byte lineBuffer[160];
//        drawBackground(gpu, LCDC, line, lineBuffer);
        drawSprites(gpu, LCDC, line, lineBuffer);
    }
}

void drawWindow(GPU* gpu, byte LCDC);
void render(byte* buffer);
void drawFrame(GPU* gpu) {
    byte LCDC = readData(gpu->LCDC);
    byte enable = (LCDC >> 7) & 0x1;
    if (enable == 1) {
        drawWindow(gpu, LCDC);
    }
    for(int i = 0; i < 64; i++){
		gpu->usedCache[i] = -1;
	}
    render(gpu->buffer);
}

int getSignedValue(byte v);
void readTileData(int tileIndex, word dataStart, int tileSize, byte* tileData);
void drawTileLine(GPU* gpu, byte* tileData, int line, int xflip, int yflip);
void copyBGTileLine(byte* lineBuffer, byte* tileBuffer, int x);
void copyLineToBuffer(GPU* gpu, byte* lineBuffer, int line);
void drawBackground(GPU* gpu, byte LCDC, int line, byte* lineBuffer) {
    if ((LCDC & 0x1) == 0) {
        return;
    }

    word mapStart = ((LCDC >> 3) & 0x1) ? gpu->START_1 : gpu->START_0;

    word dataStart;
    bool signedIndex = false;

    if (((LCDC >> 4) & 0x1)) {
        dataStart = 0x8000;
    } else {
        dataStart = 0x8800;
        signedIndex = true;
    }

    byte bgx = readData(gpu->SCX);
    byte bgy = readData(gpu->SCY);
    int tileLine = ((line + bgy) & 7);

    // browse BG tilemap for the line to render
    int tileRow = ((((bgy + line) / 8) | 0) & 0x1F);
    int firstTile = ((bgx / 8) | 0) + 32 * tileRow;
    int lastTile = firstTile + 160 / 8 + 1;
    if ((lastTile & 0x1F) < (firstTile & 0x1F)) {
        lastTile -= 32;
    }
    int x = (firstTile & 0x1F) * 8 - bgx; // x position of the first tile's leftmost pixel
    for (int i = firstTile; i != lastTile; i++) {
        if ((i & 0x1F) == 0) {
            i -= 32;
        }
        byte tileIndex = readData(i + mapStart);

        if (signedIndex == true) {
            int tileIndex = getSignedValue(tileIndex) + 128;
        }


        if(tileIndex >= 64){
        	printf("TILE INDEX IS %d", tileIndex);
        	//exit(-1);
        	continue;
        }
        bool used = false;
        for(int u = 0; u < 64; u++){
        	if(gpu->usedCache[u] == tileIndex) {
        		used = true;
        		break;
        	}
        }
        byte tileData[16];
        if(used == true){
        	for(int k = 0; k < 16; k++) {
        		tileData[k] = gpu->bgTileCache[tileIndex][k];
        	}
        }else {

        	byte td[16];
        	readTileData(tileIndex, dataStart, 16, td);
        	for(int k = 0; k < 16; k++){
        		gpu->bgTileCache[tileIndex][k] = td[k];
        	}
        	for(int a = 0; a < 64; a++){
				if(gpu->usedCache[a] == -1) {
					gpu->usedCache[a] = tileIndex;
					break;
				}
			}
        	for(int k = 0; k < 16; k++) {
				tileData[k] = td[k];
			}
        }

        drawTileLine(gpu, tileData, tileLine, 0, 0);
        copyBGTileLine(lineBuffer, gpu->tileBuffer, x);
        x += 8;
    }

    copyLineToBuffer(gpu, lineBuffer, line);
}

// Copy a tile line from a tileBuffer to a line buffer, at a given x position
void copyBGTileLine(byte* lineBuffer, byte* tileBuffer, int x) {
    // copy tile line to buffer
    for (int k = 0; k < 8; k++, x++) {
        if (x < 0 || x >= 160) continue;
        lineBuffer[x] = tileBuffer[k];
    }
}

void getPalette(byte paletteByte, byte* palette);
void drawPixel(GPU* gpu, int x, int y, byte color);
void copyLineToBuffer(GPU* gpu, byte* lineBuffer, int line) {
	byte bgPalette[4];
    getPalette(readData(gpu->BGP), bgPalette);

    for (int x = 0; x < 160; x++) {
        byte color = lineBuffer[x];
        drawPixel(gpu, x, line, bgPalette[color]);
    }
}

// Write a line of a tile (8 pixels) into a buffer array
void drawTileLine(GPU* gpu, byte* tileData, int line, int xflip, int yflip) {
    xflip = xflip | 0;
    yflip = yflip | 0;
    int l = (yflip == 1) ? 7 - line : line;
    int byteIndex = l * 2;
    byte b1 = tileData[byteIndex++];
    byte b2 = tileData[byteIndex++];

    int offset = 8;
    for (int pixel = 0; pixel < 8; pixel++) {
        offset--;
        int mask = (1 << offset);
        byte colorValue = ((b1 & mask) >> offset) + ((b2 & mask) >> offset)*2;
        int p = (xflip == 1) ? offset : pixel;
        gpu->tileBuffer[p] = colorValue;
    }
}

typedef struct {
    byte x;
    byte y;
    byte index;
    byte flags;
}Sprite;

typedef struct {
    byte color;
    byte palette;
}SpriteLine;

void copySpriteTileLine(SpriteLine* lineBuffer, byte* tileBuffer, int x, byte palette);
void copySpriteLineToBuffer(GPU* gpu, SpriteLine* spriteLineBuffer, int line);
void drawSprites(GPU* gpu, byte LCDC, int line, byte* lineBuffer) {
    if (((LCDC >> 1) & 0x1) == 0) {
        return;
    }

    int spriteHeight = ((LCDC >> 2) & 0x1) == 1 ? 16 : 8;

    Sprite sprites[9];
    int counter = 0;

    for (word i = gpu->OAM_START; i < gpu->OAM_END && counter < 9; i += 4) {
        byte y = readData(i);
        byte x = readData(i+1);
        byte index = readData(i+2);
        byte flags = readData(i+3);

        if (y - 16 > line || y - 16 < line - spriteHeight) {
            continue;
        }

        Sprite s;
        s.x = x;
        s.y = y;
        s.index = index;
        s.flags = flags;

        sprites[counter] = s;
        counter++;
    }

    if (counter == 0) return;

    // cache object to store read tiles from this frame
//    byte cacheTile[100][16];
    SpriteLine spriteLineBuffer[160];

    for (int i = 0; i < 9; i++) {
        Sprite sprite = sprites[i];
        int tileLine = line - sprite.y + 16;
        byte paletteNumber = (sprite.flags >> 4) & 0x1;
        int xflip = (sprite.flags >> 5) & 0x1;
        int yflip = (sprite.flags >> 6) & 0x1;
        byte tileData[spriteHeight * 2];
        readTileData(sprite.index, 0x8000, spriteHeight * 2, tileData);

        drawTileLine(gpu, tileData, tileLine, xflip, yflip);
        copySpriteTileLine(spriteLineBuffer, gpu->tileBuffer, sprite.x - 8, paletteNumber);
    }

    copySpriteLineToBuffer(gpu, spriteLineBuffer, line);
};

void copySpriteTileLine(SpriteLine* lineBuffer, byte* tileBuffer, int x, byte palette) {
    // copy tile line to buffer
    for (int k = 0; k < 8; k++, x++) {
        if (x < 0 || x >= 160 || tileBuffer[k] == 0) continue;
        lineBuffer[x].color = tileBuffer[k];
    	lineBuffer[x].palette = palette;
    }
}
void copySpriteLineToBuffer(GPU* gpu, SpriteLine* spriteLineBuffer, int line) {
    byte spritePalettes[2][4];
    getPalette(readData(gpu->OBP0), spritePalettes[0]);
    getPalette(readData(gpu->OBP1), spritePalettes[1]);

    for (int x = 0; x < 160; x++) {
//        if (spriteLineBuffer[x] == 0) continue; TODO: DOn;t know how to implement Maybe its fine
        byte color = spriteLineBuffer[x].color;
        if (color == 0) continue;
        byte paletteNumber = spriteLineBuffer[x].palette;
        drawPixel(gpu, x, line, spritePalettes[paletteNumber][color]);
    }
}

void drawTile(byte* tileData, int x, int y, byte* buffer, int bufferWidth, int xflip, int yflip, int spriteMode) {
    xflip = xflip | 0;
    yflip = yflip | 0;
    spriteMode = spriteMode | 0;
    int byteIndex = 0;
    for (int line = 0; line < 8; line++) {
        int l = yflip == 1 ? 7 - line : line;
        byte b1 = tileData[byteIndex++];
        byte b2 = tileData[byteIndex++];

        for (int pixel = 0; pixel < 8; pixel++) {
            int mask = (1 << (7-pixel));
            byte colorValue = ((b1 & mask) >> (7-pixel)) + ((b2 & mask) >> (7-pixel))*2;
            if (spriteMode && colorValue == 0) continue;
            int p = xflip == 1 ? 7 - pixel : pixel;
            int bufferIndex = (x + p) + (y + l) * bufferWidth;
            buffer[bufferIndex] = colorValue;
        }
    }
}

void readTileData(int tileIndex, word dataStart, int tileSize, byte* tileData) {
    word tileAddressStart = dataStart + (tileIndex * 0x10);
    int counter = 0;
    for (int i = tileAddressStart; i < tileAddressStart + tileSize; i++) {
        tileData[counter] = readData(i);
        counter++;
    }

}

void drawWindow(GPU* gpu, byte LCDC) {
    if (((LCDC >> 5) & 0x1) == 0) {
        return;
    }

    byte buffer[65536];
    word mapStart = (((LCDC >> 6) & 0x1) == 1) ? gpu->START_1 : gpu->START_0;

    word dataStart;
    bool signedIndex = false;
    if (((LCDC >> 4) & 0x1) == 1) {
        dataStart = 0x8000;
    } else {
        dataStart = 0x8800;
        signedIndex = true;
    }

    // browse Window tilemap
    for (int i = 0; i < gpu->LENGTH; i++) {
        byte tileIndex = readData(i + mapStart);

        if (signedIndex) {
            tileIndex = getSignedValue(tileIndex) + 128;
        }
        byte tileData[16];
        readTileData(tileIndex, dataStart, 16, tileData);
        int x = i % gpu->WIDTH;
        int y = (i / gpu->WIDTH) | 0;
        drawTile(tileData, x * 8, y * 8, buffer, 256, 0, 0, 0);
    }

    byte wx = readData(gpu->WX) - 7;
    byte wy = readData(gpu->WY);
    for (int x = (0 > -wx) ? 0 : -wx; x < (160 < 160 - wx) ? 160 : 160 - wx; x++) {
        for (int y = (0 > -wy) ? 0 : -wy; y < (144 < 144 - wy) ? 144 : 144 - wy; y++) {
            byte color = buffer[(x & 255) + (y & 255) * 256];
            drawPixel(gpu, x + wx, y + wy, color);
        }
    }
}

void drawPixel(GPU* gpu, int x, int y, byte color) {
    gpu->buffer[y * 160 + x] = color;
}

byte getPixel(GPU* gpu, int x, int y) {
    return gpu->buffer[y * 160 + x];
};

// Get the palette mapping from a given palette byte as stored in memory
// A palette will map a tile color to a final palette color index
// used with Screen.colors to get a shade of grey
void getPalette(byte paletteByte, byte* palette) {
    int counter = 0;
    for (int i = 0; i < 8; i += 2) {
        byte shade = (paletteByte & (3 << i)) >> i;
        palette[counter] = shade;
        counter++;
    }
}


//byte buffer[160 *144];
//void clearBuffer() {
//
//}
//void setPixel(unsigned x, unsigned y, byte color) {
//	buffer[(y - 8) * 160 + (x)] = color;
//
//}

void render(byte* buffer) {
	int start = 0;
	while (screenBase <= 0x04003fff) {
		int num = 0;
		for(int i = 0; i < 16; i++) {
			num = num << 2;
			num += (buffer[start + i] & 0x3);
		}
		*screenBase = num;
		screenBase += 1;
		start += 16;
	}
}



//END PPU





// Timer Start
word DIV_addr = 0xFF04;
word TIMA_addr = 0xFF05;
word TMA_addr = 0xFF06;
word TAC_addr = 0xFF07;


void initTimer(Timer* timer) {
	timer->time = 0;
	timer->div_time = 0;
}

void updateTimer(Timer* timer, int elapsed) {
	if(!(readData(TAC_addr) & 0x4)) {
		return;
	}
	timer->time += elapsed;
	int threshold = 64;
	switch(readData(TAC_addr) & 3) {
	case 0:
		threshold = 64;
		break;
	case 1:
		threshold = 1;
		break;
	case 2:
		threshold = 4;
		break;
	case 3:
		threshold = 16;
		break;
	}
	threshold *= 16;
	while(timer->time >= threshold) {
		timer->time -= threshold;
		byte cur_tima = readData(TIMA_addr) + 1;
		bool hasOverflown = false;
		if(cur_tima == 0x00) {
			hasOverflown = true;
		}
		writeData(TIMA_addr, cur_tima);
		if(hasOverflown) {
			writeData(TIMA_addr, readData(TMA_addr));
			requestInterrupt(TIMER);
		}
	}
}

void updateDiv(Timer* timer, int elapsed) {
	int threshold = 256;
	timer->div_time += elapsed;
	if(timer->div_time > threshold) {
		timer->div_time -= threshold;
		byte cur_div = readData(DIV_addr);
		writeData(DIV_addr, cur_div & 0xFF);
	}
}

void update(Timer* timer, int elapsed) {
	updateDiv(timer, elapsed);
	updateTimer(timer, elapsed);
}

void resetDiv(Timer* timer) {
	timer->div_time = 0;
	writeData(DIV_addr, 0);
}


//Timer End

//CPU start



void (*opcodeArray[256])(CPU*);

void init_CPU(CPU* cpu) {
	cpu->IME_flag = true;
	cpu->is_halted = false;
	cpu->is_paused = false;
	cpu->usingBootROM = false;
	initTimer(&cpu->timer);

}

void reset_CPU(CPU* cpu) {
	cpu->registers.sp = 0xFFFE;
	resetMemory();
}

int get_RAM_size() {
	int size = 0;
	switch(readData(0x149)) {
		case 1:
			size = 2048;
			break;
		case 2:
			size = 2048 * 4;
			break;
		case 3:
			size = 2048 * 16;
			break;
	}

	return size;
}

void getGameName() {
	char name[11];
	word addr = 0x134;
	for(int i = 0; i < 10; i++) {
		name[i] = readData(addr);
		addr += 1;
	}
	printf("%s\n", name);
}
void checkInterrupt(CPU* cpu);
void fetch(CPU* cpu);
void run(CPU* cpu) {
	if(cpu->usingBootROM == true) {
		cpu->registers.pc = 0x0000;
	}else {
		cpu->registers.pc = 0x0100;
	}
	fetch(cpu);
}

byte fetchOpcode(CPU* cpu);
void executeOpcode(CPU* cpu, byte opcode);

void updateInput() {
	volatile unsigned int *KEY_PIO = (unsigned int*)0x04005120;
//	printf("%d\n", *KEY_PIO);
	if(*KEY_PIO  != 3) {
		printf("Executing Input Interrupt START\n");
		requestInterrupt(HILO);
	}
}

void fetch(CPU* cpu) {
	bool vBlank = false;
	while(1) {
		int oldInstruction = cpu->clock.c;
		if(cpu->is_halted == false){
			byte opcode = fetchOpcode(cpu);
			executeOpcode(cpu, opcode);
			cpu->registers.f &= 0xF0;
		}else {
			cpu->clock.c += 4;
		}

		int elapsed = cpu->clock.c - oldInstruction;
		vBlank = gpuUpdate(cpu->gpu, elapsed);
		updateTimer(&(cpu->timer), elapsed);
		//TODO: Update Input
		updateInput();
		checkInterrupt(cpu);
	}
	cpu->clock.c = 0;
}

byte fetchOpcode(CPU* cpu) {
	return readData(cpu->registers.pc++);
}

void executeOpcode(CPU* cpu, byte opcode) {
	if(opcodeArray[opcode] == NULL){
		printf("Undefined Opcode: %02x\n", opcode);
		exit(1);
	}
	printf("Executing Opcode %04x: %02x   | B: %02x   L: %02x\n",cpu->registers.pc - 1, opcode, cpu->registers.b, cpu->registers.l);
	(*opcodeArray[opcode])(cpu);
}



byte rr8(CPU* cpu, reg_name r) {
	switch(r) {
	case A:
		return cpu->registers.a;
	case B:
		return cpu->registers.b;
	case C:
		return cpu->registers.c;
	case D:
		return cpu->registers.d;
	case E:
		return cpu->registers.e;
	case F:
		return cpu->registers.f;
	case H:
		return cpu->registers.h;
	case L:
		return cpu->registers.l;
	default:
		return 0x00;
	}
	return 0x00;
}
word rr16(CPU* cpu, reg_name r) {
	word out = 0x0000;
	switch(r) {
	case AF:
		out += cpu->registers.a;
		out = out << 8;
		out += cpu->registers.f;
		return out;
	case BC:
		out += cpu->registers.b;
		out = out << 8;
		out += cpu->registers.c;
		return out;
	case DE:
		out += cpu->registers.d;
		out = out << 8;
		out += cpu->registers.e;
		return out;
	case HL:
		out += cpu->registers.h;
		out = out << 8;
		out += cpu->registers.l;
		return out;
	case SP:
		return cpu->registers.sp;
	case PC:
		return cpu->registers.pc;
	default:
		return out;
	}
	return out;
}

void wr8(CPU* cpu, reg_name r, byte value) {
	switch(r) {
	case A:
		cpu->registers.a = value;
		break;
	case B:
		cpu->registers.b = value;
		break;
	case C:
		cpu->registers.c = value;
		break;
	case D:
		cpu->registers.d = value;
		break;
	case E:
		cpu->registers.e = value;
		break;
	case F:
		cpu->registers.f = value;
		break;
	case H:
		cpu->registers.h = value;
		break;
	case L:
		cpu->registers.l = value;
		break;
	default:
		break;
	}
}

void wr16(CPU* cpu, reg_name r, word value) {
	byte p1 = 0x00;
	byte p2 = 0x00;
	p1 = (value & 0xFF00) >> 8;
	p2 = (value & 0x00FF);
	switch(r) {
	case AF:
		cpu->registers.a = p1;
		cpu->registers.f = p2;
		break;
	case BC:
		cpu->registers.b = p1;
		cpu->registers.c = p2;
		break;
	case DE:
		cpu->registers.d = p1;
		cpu->registers.e = p2;
		break;
	case HL:
		cpu->registers.h = p1;
		cpu->registers.l = p2;
		break;
	case SP:
		cpu->registers.sp = value;
		break;
	case PC:
		cpu->registers.pc = value;
		break;
	default:
		break;
	}
}

void halt(CPU* cpu) {
	cpu->is_halted = true;
}

void unHalt(CPU* cpu) {
	cpu->is_halted = false;
}

void pause(CPU* cpu) {
	cpu->is_paused = true;
}

void unPause(CPU* cpu) {
	if(cpu->is_paused == true) {
		cpu->is_paused = false;
		fetch(cpu);
	}
}

void RSTn(CPU* cpu, word n);

void i0(CPU* cpu){RSTn(cpu, 0x40);}
void i1(CPU* cpu){RSTn(cpu, 0x48);}
void i2(CPU* cpu){RSTn(cpu, 0x50);}
void i3(CPU* cpu){RSTn(cpu, 0x58);}
void i4(CPU* cpu){RSTn(cpu, 0x60);}


bool isInterruptEnable(interupt_name index);
void disableInterrupts(CPU* cpu);
void checkInterrupt(CPU* cpu) {
	if(cpu->IME_flag == false) return;

	for(unsigned i = 0; i < 5; i++) {
		byte IFval = readData(0xFF0F);

		if (((IFval >> i) & 1) && isInterruptEnable(i)) {
			IFval &= (0xFF - (1<<i));
			writeData(0xFF0F, IFval);
			disableInterrupts(cpu);
			cpu->clock.c += 4;
			printf("INTERUPT!! %d\n", i);
			switch(i){
			case 0:
				i0(cpu);
				break;
			case 1:
				i1(cpu);
				break;
			case 2:
				i2(cpu);
				break;
			case 3:
				i3(cpu);
				break;
			case 4:
				i4(cpu);
				break;
			}
			break;
		}
	}
}

void requestInterrupt(interupt_name index) {
	byte IFval = readData(0xFF0F);
	IFval |= (1 << index);
	writeData(0xFF0F, IFval);
//	unHalt();
}

bool isInterruptEnable(interupt_name index) {
	if ((readData(0xFFFF) >> index & 1) != 0) {
		return true;
	}
	else {
		return false;
	}
}

void enableInterrupts(CPU* cpu) {
	cpu->IME_flag = true;
}

void disableInterrupts(CPU* cpu) {
	cpu->IME_flag = false;
}

//End CPU
//Start Instructions
word readAddr(CPU* cpu, reg_name r1, reg_name r2) {
	word out = 0x0000;
	out += rr8(cpu, r1);
	out = out << 8;
	out += rr8(cpu, r2);
	return out;
}
int getSignedValue(byte v) {
	int val = 0;
	if(v & 0x80) {
//		printf("If\n");
		val += v;
		val -= 256;
	}else {
//		printf("Else\n");
		val += v;
	}
	return val;
}
void _LDav(CPU* cpu, word addr, byte val);
void LDrrnn(CPU* cpu, reg_name r1, reg_name r2) {
	wr8(cpu, r2, readData(cpu->registers.pc));
	wr8(cpu, r1, readData(cpu->registers.pc + 1));
	cpu->registers.pc += 2;
	cpu->clock.c += 12;
}
void LDrrar(CPU* cpu, reg_name r1, reg_name r2, reg_name r3) {
	_LDav(cpu, readAddr(cpu, r1, r2), rr8(cpu, r3));
	cpu->clock.c += 8;
}
void LDrrra(CPU* cpu, reg_name r1, reg_name r2, reg_name r3) {
	wr8(cpu, r1, readData(readAddr(cpu, r2, r3)));
	cpu->clock.c += 8;
}
void LDrn(CPU* cpu, reg_name r1){
	wr8(cpu, r1, readData(cpu->registers.pc++));
	cpu->clock.c += 8;
}
void LDrr(CPU* cpu, reg_name r1, reg_name r2) {
	wr8(cpu, r1, rr8(cpu, r2));
	cpu->clock.c += 4;
}
void LDrar(CPU* cpu, reg_name r1, reg_name r2) {
	writeData(0xFF00 + rr8(cpu, r1), rr8(cpu, r2));
	cpu->clock.c += 8;
}
void LDrra(CPU* cpu, reg_name r1, reg_name r2) {
	wr8(cpu, r1, readData(0xFF00 + rr8(cpu, r2)));
	cpu->clock.c += 8;
}
void LDspnn(CPU* cpu) {
	wr16(cpu, SP, 0x0000 + readData((cpu->registers.pc + 1) << 8) + readData(cpu->registers.pc));
	cpu->registers.pc += 2;
	cpu->clock.c += 12;
}
void LDsprr(CPU* cpu, reg_name r1, reg_name r2) {
	wr16(cpu, SP, readAddr(cpu, r1, r2));
	cpu->clock.c += 8;
}
void LDnnar(CPU* cpu, reg_name r1) {
	word addr = ((0x0000 + readData(cpu->registers.pc + 1)) << 8) + readData(cpu->registers.pc);
	writeData(addr, rr8(cpu, r1));
	cpu->registers.pc += 2;
	cpu->clock.c += 16;
}
void LDrnna(CPU* cpu, reg_name r1) {
	word addr = ((0x0000 + readData(cpu->registers.pc + 1)) << 8) + readData(cpu->registers.pc);
	wr8(cpu, r1, readData(addr));
	cpu->registers.pc += 2;
	cpu->clock.c += 16;
}
void LDrrspn(CPU* cpu, reg_name r1, reg_name r2) {
	int rel = readData(cpu->registers.pc++);
	rel = getSignedValue(rel);
	int val = cpu->registers.sp + rel;
	int c = (cpu->registers.sp & 0xFF) + (rel & 0xFF) > 0xFF;
	int h = (cpu->registers.sp & 0xF) + (rel & 0xF) > 0xF;
	val &= 0xFFFF;
	int f = 0;
	if(h) {
		f |= 0x20;
	}
	if(c) {
		f |= 0x10;
	}
	wr8(cpu, F, f);
	wr8(cpu, r1, val >> 8);
	wr8(cpu, r2, val & 0xFF);
	cpu->clock.c += 12;
}
void LDnnsp(CPU* cpu) {
	word addr = 0;
	addr += readData(cpu->registers.pc++);
	word temp = readData(cpu->registers.pc++);
	temp = temp << 8;
	addr += temp;
	_LDav(cpu, addr, 0xFF & rr16(cpu, SP));
	_LDav(cpu, addr + 1, (rr16(cpu, SP) >> 8));
	cpu->clock.c += 20;
}
void LDrran(CPU* cpu, reg_name r1, reg_name r2) {
	word addr = readAddr(cpu, r1, r2);
	_LDav(cpu, addr, readData(cpu->registers.pc++));
	cpu->clock.c += 12;
}
void _LDav(CPU* cpu, word addr, byte val) {
	writeData(addr, val);
}
void LDHnar(CPU* cpu, reg_name r1) {
	writeData(0xFF00 + readData(cpu->registers.pc++), rr8(cpu, r1));
	cpu->clock.c += 12;
}
void LDHrna(CPU* cpu, reg_name r1) {
	wr8(cpu, r1, readData(0xFF00 + readData(cpu->registers.pc++)));
	cpu->clock.c += 12;
}
void INCrr(CPU* cpu, reg_name r1, reg_name r2) {
	wr8(cpu, r2, (rr8(cpu, r2) + 1) & 0xFF);
	if(rr8(cpu, r2) == 0) {
		wr8(cpu, r1, (rr8(cpu, r1) + 1) & 0xFF);
	}
	cpu->clock.c += 8;
}
void INCrra(CPU* cpu, reg_name r1, reg_name r2) {
	word addr = readAddr(cpu, r1, r2);
	byte val = (readData(addr) + 1) & 0xFF;
	int z = val == 0;
	int h = (readData(addr) & 0xF) + 1 > 0xF;
	writeData(addr, val);
	cpu->registers.f &= 0x10;
	if(h) cpu->registers.f |= 0x20;
	if(z) cpu->registers.f |= 0x80;
	cpu->clock.c += 12;
}
void INCsp(CPU* cpu) {
	wr16(cpu, SP, cpu->registers.sp + 1);
	cpu->registers.sp &= 0xFFFF;
	cpu->clock.c += 8;
}
void INCr(CPU* cpu, reg_name r1) {
	int h = ((rr8(cpu, r1) & 0xF) + 1) & 0x10;
	wr8(cpu, r1, (rr8(cpu, r1) + 1) & 0xFF);
	int z = rr8(cpu, r1) == 0;
	cpu->registers.f &= 0x10;
	if(h) cpu->registers.f |= 0x20;
	if(z) cpu->registers.f |= 0x80;
	cpu->clock.c += 4;
}
void DECrr(CPU* cpu, reg_name r1, reg_name r2) {
	wr8(cpu, r2, (rr8(cpu, r2) - 1) & 0xFF);
	if(rr8(cpu, r2) == 0xFF) wr8(cpu, r1, (rr8(cpu, r1) - 1) & 0xFF);
	cpu->clock.c += 8;
}
void DECsp(CPU* cpu) {
	wr16(cpu, SP, rr16(cpu, SP) - 1);
	cpu->registers.sp &= 0xFFFF;
	cpu->clock.c += 8;
}
void DECr(CPU* cpu, reg_name r1) {
	int h = (rr8(cpu, r1) & 0xF) < 1;
	wr8(cpu, r1, (rr8(cpu, r1) - 1) & 0xFF);
	int z = rr8(cpu, r1) == 0;
	cpu->registers.f &= 0x10;
	cpu->registers.f |= 0x40;
	if(h) cpu->registers.f |= 0x20;
	if(z) cpu->registers.f |= 0x80;
	cpu->clock.c += 4;
}
void DECrra(CPU* cpu, reg_name r1, reg_name r2) {
	word addr = readAddr(cpu, r1, r2);
	byte val = (readData(addr) - 1) & 0xFF;
	int z = val == 0;
	int h = (readData(addr) * 0xF) < 1;
	writeData(addr, val);
	cpu->registers.f &= 0x10;
	cpu->registers.f |= 0x40;
	if(h) cpu->registers.f |= 0x20;
	if(z) cpu->registers.f |= 0x80;
	cpu->clock.c += 12;
}
void _ADDrrn(CPU* cpu, reg_name r1, reg_name r2, word n);
void _ADDrn(CPU* cpu, reg_name r1, byte n);
void ADDrr(CPU* cpu, reg_name r1, reg_name r2) {
	byte n = rr8(cpu, r2);
	_ADDrn(cpu, r1, n);
	cpu->clock.c += 4;
}
void ADDrn(CPU* cpu, reg_name r1) {
	byte n = readData(cpu->registers.pc++);
	_ADDrn(cpu, r1, n);
	cpu->clock.c += 8;
}
void _ADDrn(CPU* cpu, reg_name r1, byte n) {
	int h = ((rr8(cpu, r1) & 0xF) + (n & 0xF)) & 0x10;
	int newVal = rr8(cpu, r1);
	newVal += n;
	int c = 0;
	if(newVal > 255) {
		c = 1;
	}
	wr8(cpu, r1, rr8(cpu, r1) + n);
	byte f = 0;
	if(rr8(cpu, r1) == 0) f |= 0x80;
	if(h) f |= 0x20;
	if(c) f |= 0x10;
	wr8(cpu, F, f);
}
void ADDrrrr(CPU* cpu, reg_name r1, reg_name r2, reg_name r3, reg_name r4) {
	word n = rr8(cpu, r3);
	n = (n << 8) + rr8(cpu, r4);
	_ADDrrn(cpu, r1, r2, n);
	cpu->clock.c += 8;
}
void ADDrrsp(CPU* cpu, reg_name r1, reg_name r2) {
	_ADDrrn(cpu, r1, r2, rr16(cpu, SP));
	cpu->clock.c += 8;
}
void ADDspn(CPU* cpu) {
	byte v = readData(cpu->registers.pc++);
	v = getSignedValue(v);
	int c = ((cpu->registers.sp & 0xFF) + (v & 0xFF)) > 0xFF;
	int h = ((cpu->registers.sp & 0xF) + (v & 0xF)) > 0xF;
	byte f = 0;
	if(h) f |= 0x20;
	if(c) f |= 0x10;
	wr8(cpu, F, f);
	wr16(cpu, SP, (rr16(cpu, SP) + v) & 0xFFFF);
	cpu->clock.c += 16;
}
void _ADDrrn(CPU* cpu, reg_name r1, reg_name r2, word n) {
	word v1 = rr8(cpu, r1);
	v1 = (v1 << 8) + rr8(cpu, r2);
	word v2 = n;
	int res = v1 + v2;
	int c = res & 0x10000;
	int h = ((v1 & 0xFFF) + (v2 & 0xFFF)) & 0x1000;
	int z = cpu->registers.f & 0x80;
	res &= 0xFFFF;
	wr8(cpu, r2, res & 0xFF);
	res = res >> 8;
	wr8(cpu, r1, res & 0xFF);
	byte f = 0;
	if (z) f |= 0x80;
	if (h) f |= 0x20;
	if (c) f |= 0x10;
	wr8(cpu, F, f);
}
void _ADCrn(CPU* cpu, reg_name r1, byte n);
void ADCrr(CPU* cpu, reg_name r1, reg_name r2) {
	byte n = rr8(cpu, r2);
	_ADCrn(cpu, r1, n);
	cpu->clock.c += 4;
}
void ADCrn(CPU* cpu, reg_name r1) {
	byte n = readData(cpu->registers.pc++);
	_ADCrn(cpu, r1, n);
	cpu->clock.c += 8;
}
void _ADCrn(CPU* cpu, reg_name r1, byte n) {
	int c = rr8(cpu, F) & 0x10 ? 1 : 0;
	int h = ((rr8(cpu, r1) & 0xF) + (n & 0xF) + c) & 0x10;
	int prev = rr8(cpu, r1);
	prev += n;
	prev += c;
	wr8(cpu, r1, rr8(cpu, r1) + n + c);
	if(prev > 255) {
		c = 1;
	}
	byte f = 0;
	if (rr8(cpu, r1) == 0) f |= 0x80;
	if (h) f |= 0x20;
	if (c) f |= 0x10;
	wr8(cpu, F, f);
}
void ADCrrra(CPU* cpu, reg_name r1, reg_name r2, reg_name r3) {
	byte n = readData(readAddr(cpu, r2, r3));
	_ADCrn(cpu, r1, n);
	cpu->clock.c += 8;
}
void ADDrrra(CPU* cpu, reg_name r1, reg_name r2, reg_name r3) {
	byte v = readData(readAddr(cpu, r2, r3));
	int h = ((rr8(cpu, r1) & 0xF) + (v & 0xF)) & 0x10;
	int c = 0;
	int prev = rr8(cpu, r1);
	prev += v;
	wr8(cpu, r1, rr8(cpu, r1) + v);
	if(prev > 255) {
		c = 1;
	}
	byte f = 0;
	if (rr8(cpu, r1) == 0) f |= 0x80;
	if (h) f |= 0x20;
	if (c) f |= 0x10;
	wr8(cpu, F, f);
	cpu->clock.c += 8;
}
void _SUBn(CPU* cpu, byte n);
void SUBr(CPU* cpu, reg_name r1) {
	byte n = rr8(cpu, r1);
	_SUBn(cpu, n);
	cpu->clock.c += 4;
}
void SUBn(CPU* cpu) {
	byte n = readData(cpu->registers.pc++);
	_SUBn(cpu, n);
	cpu->clock.c += 8;
}
void SUBrra(CPU* cpu, reg_name r1, reg_name r2) {
	byte n = readData(readAddr(cpu, r1, r2));
	_SUBn(cpu, n);
	cpu->clock.c += 8;
}
void _SUBn(CPU* cpu, byte n) {
	int c = rr8(cpu, A) < n;
	int h = (rr8(cpu, A) & 0xF) < (n & 0xF);
	wr8(cpu, A, rr8(cpu, A) - n);
	int z = rr8(cpu, A) == 0;
	byte f = 0x40;
	if (z) f |= 0x80;
	if (h) f |= 0x20;
	if (c) f |= 0x10;
	wr8(cpu, F, f);
}
void _SBCn(CPU* cpu, byte n);
void SBCn(CPU* cpu) {
	byte n = readData(cpu->registers.pc++);
	_SBCn(cpu, n);
	cpu->clock.c += 8;
}
void SBCr(CPU* cpu, reg_name r1) {
	byte n = rr8(cpu, r1);
	_SBCn(cpu, n);
	cpu->clock.c += 4;
}
void SBCrra(CPU* cpu, reg_name r1, reg_name r2) {
	word addr = rr8(cpu, r1);
	addr = (addr << 8) + rr8(cpu, r2);
	byte v = readData(addr);
	_SBCn(cpu, v);
	cpu->clock.c += 8;
}
void _SBCn(CPU* cpu, byte n) {
	int carry = rr8(cpu, F) & 0x10 ? 1 : 0;
	int c = rr8(cpu, A) < (carry + n);
	int h = (rr8(cpu, A) & 0xF) < (carry + (n & 0xF));
	wr8(cpu, A, rr8(cpu, A) - n - carry);
	cpu->registers.a &= 0xFF;
	int z = rr8(cpu, A) == 0;
	byte f = 0x40;
	if (z) f |= 0x80;
	if (h) f |= 0x20;
	if (c) f |= 0x10;
	wr8(cpu, F, f);
}
void ORr(CPU* cpu, reg_name r1) {
	cpu->registers.a |= rr8(cpu, r1);
	wr8(cpu, F, (rr8(cpu, A) == 0) ? 0x80 : 0x00);
	cpu->clock.c += 4;
}
void ORn(CPU* cpu) {
    cpu->registers.a |= readData(cpu->registers.pc++);
    cpu->registers.f = (cpu->registers.a == 0) ? 0x80 : 0x00;
    cpu->clock.c += 8;
}

void ORrra(CPU* cpu, reg_name r1, reg_name r2) {
    cpu->registers.a |= readData(readAddr(cpu, r1, r2));
    cpu->registers.f = (cpu->registers.a == 0) ? 0x80 : 0x00;
    cpu->clock.c += 8;
}

void ANDr(CPU* cpu, reg_name r1) {
    cpu->registers.a &= rr8(cpu, r1);
    cpu->registers.f = (cpu->registers.a == 0) ? 0xA0 : 0x20;
    cpu->clock.c += 4;
}

void ANDn(CPU* cpu) {
    cpu->registers.a &= readData(cpu->registers.pc++);
    cpu->registers.f = (cpu->registers.a == 0) ? 0xA0 : 0x20;
    cpu->clock.c += 8;
}

void ANDrra(CPU* cpu, reg_name r1, reg_name r2) {
    cpu->registers.a &= readData(readAddr(cpu, r1, r2));
    cpu->registers.f = (cpu->registers.a == 0) ? 0xA0 : 0x20;
    cpu->clock.c += 8;
}

void XORr(CPU* cpu, reg_name r1) {
    cpu->registers.a ^= rr8(cpu, r1);
    cpu->registers.f = (cpu->registers.a == 0) ? 0x80 : 0x00;
    cpu->clock.c += 4;
}

void XORn(CPU* cpu) {
    cpu->registers.a ^= readData(cpu->registers.pc++);
    cpu->registers.f = (cpu->registers.a == 0) ? 0x80 : 0x00;
    cpu->clock.c += 8;
}

void XORrra(CPU* cpu, reg_name r1, reg_name r2) {
    cpu->registers.a ^= readData(readAddr(cpu, r1, r2));
    cpu->registers.f = (cpu->registers.a == 0) ? 0x80 : 0x00;
    cpu->clock.c += 8;
}

void _CPn(CPU* cpu, byte n);

void CPr(CPU* cpu, reg_name r1) {
    byte n = rr8(cpu, r1);
    _CPn(cpu, n);
    cpu->clock.c += 4;
}

void CPn(CPU* cpu) {
    byte n = readData(cpu->registers.pc++);
    _CPn(cpu, n);
    cpu->clock.c += 8;
}

void CPrra(CPU* cpu, reg_name r1, reg_name r2) {
    byte n = readData(readAddr(cpu, r1, r2));
    _CPn(cpu, n);
    cpu->clock.c += 8;
}

void _CPn(CPU* cpu, byte n) {
    int c = (cpu->registers.a < n) ? 1 : 0;
    int z = (cpu->registers.a == n) ? 1 : 0;
    int h = ((cpu->registers.a & 0xF) < (n & 0xF)) ? 1 : 0;
    byte f = 0x40;

    if (z == 1) f += 0x80;
    if (h == 1) f += 0x20;
    if (c == 1) f += 0x10;

    cpu->registers.f = f;
}

void RRCr(CPU* cpu, reg_name r1) {
    cpu->registers.f = 0;
    byte out = rr8(cpu, r1) & 0x01;
    if (out == 0) cpu->registers.f |= 0x10;
    wr8(cpu, r1, ((rr8(cpu, r1) >> 1) | (out * 0x80)));
    if (rr8(cpu, r1) == 0) cpu->registers.f |= 0x80;
    cpu->clock.c += 4;
}

void RRCrra(CPU* cpu, reg_name r1, reg_name r2) {
    word addr = readAddr(cpu, r1, r2);
    cpu->registers.f = 0;
    byte out = readData(addr) & 0x01;
    if(out == 1) cpu->registers.f |= 0x10;
    writeData(addr, (readData(addr) >> 1) | (out * 0x80));
    if (readData(addr) == 0) cpu->registers.f |= 0x80;
    cpu->clock.c += 12;
}

void RLCr(CPU* cpu, reg_name r1) {
    cpu->registers.f = 0;
    byte out = (rr8(cpu, r1) & 0x80) ? 1 : 0;
    if(out == 1) cpu->registers.f |= 0x10;
    wr8(cpu, r1, (((0x0000 + rr8(cpu, r1)) << 1) + out) & 0xFF);
    if (rr8(cpu, r1) == 0) cpu->registers.a |= 0x80;
    cpu->clock.c += 4;
}

void RLCrra(CPU* cpu, reg_name r1, reg_name r2) {
    word addr = readAddr(cpu, r1, r2);
    cpu->registers.f = 0;
    byte out = (readData(addr) & 0x80) ? 1 : 0;
    if(out == 1) cpu->registers.f |= 0x10;
    writeData(addr, (((0x0000 + readData(addr)) << 1) + out) & 0xFF);
    if (readData(addr) == 0) cpu->registers.f |= 0x80;
    cpu->clock.c += 12;
}

void RLr(CPU* cpu, reg_name r1) {
	byte c = (cpu->registers.f & 0x10) ? 1 : 0;
	cpu->registers.f = 0;
	byte out = rr8(cpu, r1) & 0x80;
	if (out != 0) {
		cpu->registers.f |= 0x10;
	}
	else {
		cpu->registers.f &= 0xEF;
	}
	wr8(cpu, r1, (((0x0000 + rr8(cpu, r1)) << 1) + c) & 0xFF);
	if (rr8(cpu, r1) == 0) cpu->registers.f |= 0x80;
	cpu->clock.c += 4;
}

void RLrra(CPU* cpu, reg_name r1, reg_name r2) {
    word addr = readAddr(cpu, r1, r2);
    byte c = (cpu->registers.f & 0x10) ? 1 : 0;
    cpu->registers.f = 0;
    byte out = readData(addr) & 0x80;
    if (out != 0) {
    	cpu->registers.f |= 0x10;
	}
	else {
		cpu->registers.f &= 0xEF;
	}
    writeData(addr, (((0x0000 + readData(addr)) << 1) + c) & 0xFF);
    if (readData(addr) == 0) cpu->registers.f |= 0x80;
    cpu->clock.c += 12;
}

void RRr(CPU* cpu, reg_name r1) {
    byte c = (cpu->registers.f & 0x10) ? 1 : 0;
    cpu->registers.f = 0;
    byte out = rr8(cpu, r1) & 0x01;
    if (out != 0) {
		cpu->registers.f |= 0x10;
	}
	else {
		cpu->registers.f &= 0xEF;
	}
    wr8(cpu, r1, (rr8(cpu, r1) >> 1) | (c * 0x80));
    cpu->clock.c += 4;
}

void RRrra(CPU* cpu, reg_name r1, reg_name r2) {
    word addr = readAddr(cpu, r1, r2);
    byte c = (cpu->registers.f & 0x10) ? 1 : 0;
    cpu->registers.f = 0;
    byte out = readData(addr) & 0x01;
    if (out != 0) {
		cpu->registers.f |= 0x10;
	}
	else {
		cpu->registers.f &= 0xEF;
	}
    writeData(addr, (readData(addr) >> 1) | (c * 0x80));
    if (readData(addr) == 0) cpu->registers.f |= 0x80;
    cpu->clock.c += 12;
}

void SRAr(CPU* cpu, reg_name r1) {
    cpu->registers.f = 0;
    if ((rr8(cpu, r1) & 0x01) == 1) cpu->registers.f |= 0x10;
    byte msb = rr8(cpu, r1) & 0x80;
    wr8(cpu, r1, (rr8(cpu, r1) >> 1) | msb);
    if (rr8(cpu, r1) == 0) cpu->registers.f |= 0x80;
    cpu->clock.c += 4;
}

void SRArra(CPU* cpu, reg_name r1, reg_name r2) {
    word addr = readAddr(cpu, r1, r2);
    cpu->registers.f = 0;
    if ((readData(addr) & 0x01) == 1) cpu->registers.f |= 0x10;
    byte msb = readData(addr) & 0x80;
    writeData(addr, (readData(addr) >> 1) | msb);
    if (readData(addr) == 0) cpu->registers.f |= 0x80;
    cpu->clock.c += 12;
}

void SLAr(CPU* cpu, reg_name r1) {
    cpu->registers.f = 0;
    if ((rr8(cpu, r1) & 0x80) != 0) cpu->registers.f |= 0x10;
    wr8(cpu, r1, ((rr8(cpu, r1) << 1) & 0xFF));
    if (rr8(cpu, r1) == 0) cpu->registers.f |= 0x80;
    cpu->clock.c += 4;
}

void SLArra(CPU* cpu, reg_name r1, reg_name r2) {
    word addr = readAddr(cpu, r1, r2);
    cpu->registers.f = 0;
    if ((readData(addr) & 0x80) != 0) cpu->registers.f |= 0x10;
    writeData(addr, ((readData(addr) << 1) & 0xFF));
    if (readData(addr) == 0) cpu->registers.f |= 0x80;
    cpu->clock.c += 12;
}

void SRLr(CPU* cpu, reg_name r1) {
    cpu->registers.f = 0;
    if ((rr8(cpu, r1) & 0x01) != 0) cpu->registers.f |= 0x10;
    wr8(cpu, r1, ((0x0000 + rr8(cpu, r1)) >> 1));
    if (rr8(cpu, r1) == 0) cpu->registers.f |= 0x80;
    cpu->clock.c += 4;
}

void SRLrra(CPU* cpu, reg_name r1, reg_name r2) {
    word addr = readAddr(cpu, r1, r2);
    cpu->registers.f = 0;
    if ((readData(addr) & 0x01) != 0) cpu->registers.f |= 0x10;
    writeData(addr, (readData(addr) >> 1));
    if (readData(addr) == 0) cpu->registers.f |= 0x80;
    cpu->clock.c += 12;
}

void BITir(CPU* cpu, int i, reg_name r1) {
    byte mask = 1 << i;
    byte z = (rr8(cpu, r1) & mask) ? 0 : 1;
    byte f = cpu->registers.f & 0x10;
    f |= 0x20;
    if (z == 1) f |= 0x80;
    cpu->registers.f = f;
    cpu->clock.c += 4;
}

void BITirra(CPU* cpu, int i, reg_name r1, reg_name r2) {
    word addr = readAddr(cpu, r1, r2);
    byte mask = 1 << i;
    byte z = (readData(addr) & mask) ? 0 : 1;
    byte f = cpu->registers.f & 0x10;
    f |= 0x20;
    if (z == 1) f |= 0x80;
    cpu->registers.f = f;
    cpu->clock.c += 8;
}

void SETir(CPU* cpu, int i, reg_name r1) {
    byte mask = 1 << i;
    wr8(cpu, r1, rr8(cpu, r1) | mask);
    cpu->clock.c += 4;
}

void SETirra(CPU* cpu, int i, reg_name r1, reg_name r2) {
    word addr = readAddr(cpu, r1, r2);
    byte mask = 1 << i;
    writeData(addr, readData(addr) | mask);
    cpu->clock.c += 12;
}

void RESir(CPU* cpu, int i, reg_name r1) {
    byte mask = 0xFF - (1 << i);
    wr8(cpu, r1, rr8(cpu, r1) & mask);
    cpu->clock.c += 4;
}

void RESirra(CPU* cpu, int i, reg_name r1, reg_name r2) {
    word addr = readAddr(cpu, r1, r2);
    byte mask = 0xFF - (1 << i);
    writeData(addr, readData(addr) & mask);
    cpu->clock.c += 12;
}

byte _SWAPn(CPU* cpu, byte n);

void SWAPr(CPU* cpu, reg_name r1) {
    wr8(cpu, r1, _SWAPn(cpu, rr8(cpu, r1)));
    cpu->clock.c += 4;
}

void SWAPrra(CPU* cpu, reg_name r1, reg_name r2) {
    word addr = ((0x0000 + rr8(cpu, r1)) << 8) + rr8(cpu, r2);
    writeData(addr, _SWAPn(cpu, readData(addr)));
    cpu->clock.c += 12;
}

byte _SWAPn(CPU* cpu, byte n) {
    cpu->registers.f = (n == 0) ? 0x80 : 0;
    return (((n & 0xF0) >> 4) | ((n & 0x0F) << 4));
}

void JPnn(CPU* cpu) {
    wr16(cpu, PC, ((0x0000 + readData(rr16(cpu, PC) + 1)) << 8) + readData(rr16(cpu, PC)));
    cpu->clock.c += 16;
}

int testFlag(CPU* cpu, cond_code cc) {
    byte test = 1;
    byte mask = 0x10;
    if (cc == NC || cc == NZ) test = 0;
    if (cc == NZ || cc == Z) mask = 0x80;

    byte c1 = (test && (cpu->registers.f & mask)) ? 1 : 0;
    byte c2 = !(test || (cpu->registers.f & mask)) ? 1 : 0;

    return c1 || c2;
}

void JRccn(CPU* cpu, cond_code cc) {
    if (testFlag(cpu, cc) == 1) {
        byte v = readData(cpu->registers.pc++);
        int r = getSignedValue(v);
        cpu->registers.pc += r;
        cpu->clock.c += 4;
    }
    else {
        cpu->registers.pc++;
    }
    cpu->clock.c += 8;
}

void JPccnn(CPU* cpu, cond_code cc) {
    if (testFlag(cpu, cc) == 1) {
        wr16(cpu, PC, ((0x0000 + readData(cpu->registers.pc + 1)) << 8) + readData(cpu->registers.pc));
        cpu->clock.c += 4;
    }
    else {
        cpu->registers.pc += 2;
    }

    cpu->clock.c += 12;
}

void JPrr(CPU* cpu, reg_name r1, reg_name r2) {
    wr16(cpu, PC, ((0x0000 + rr8(cpu, r1)) + rr8(cpu, r2)));
    cpu->clock.c += 4;
}

void JRn(CPU* cpu) {
    byte v = readData(cpu->registers.pc++);
    v = getSignedValue(v);
    cpu->registers.pc += v;
    cpu->clock.c += 12;
}

void PUSHrr(CPU* cpu, reg_name r1, reg_name r2) {
    wr16(cpu, SP, cpu->registers.sp - 1);
    writeData(cpu->registers.sp, rr8(cpu, r1));
    wr16(cpu, SP, cpu->registers.sp - 1);
    writeData(cpu->registers.sp, rr8(cpu, r2));
    cpu->clock.c += 16;
}

void POPrr(CPU* cpu, reg_name r1, reg_name r2) {
    wr8(cpu, r2, readData(cpu->registers.sp));
    wr16(cpu, SP, cpu->registers.sp + 1);
    wr8(cpu, r1, readData(cpu->registers.sp));
    wr16(cpu, SP, cpu->registers.sp + 1);
    cpu->clock.c += 12;
}

void RSTn(CPU* cpu, word n) {
    wr16(cpu, SP, cpu->registers.sp - 1);
    writeData(cpu->registers.sp, cpu->registers.pc >> 8);
    wr16(cpu, SP, cpu->registers.sp - 1);
    writeData(cpu->registers.sp, cpu->registers.pc & 0xFF);
    wr16(cpu, PC, n);
    cpu->clock.c += 16;
}

void RET(CPU* cpu) {
    wr16(cpu, PC, readData(cpu->registers.sp));
    wr16(cpu, SP, cpu->registers.sp + 1);
    wr16(cpu, PC, (0x0000 + readData(cpu->registers.sp)) << 8);
    wr16(cpu, SP, cpu->registers.sp + 1);
    cpu->clock.c += 16;
}

void RETcc(CPU* cpu, cond_code cc) {
    if (testFlag(cpu, cc) == 1) {
        wr16(cpu, PC, readData(cpu->registers.sp));
        wr16(cpu, SP, cpu->registers.sp + 1);
        wr16(cpu, PC, (0x0000 + readData(cpu->registers.sp)) << 8);
        wr16(cpu, SP, cpu->registers.sp + 1);
        cpu->clock.c += 12;
    }
    cpu->clock.c += 8;
}

void _CALLnn(CPU* cpu);

void CALLnn(CPU* cpu) {
    _CALLnn(cpu);
    cpu->clock.c += 24;
}

void CALLccnn(CPU* cpu, cond_code cc) {
    if (testFlag(cpu, cc) == 1) {
        _CALLnn(cpu);
        cpu->clock.c += 12;
    }
    else {
        cpu->registers.pc += 2;
    }
    cpu->clock.c += 12;
}

void _CALLnn(CPU* cpu) {
    wr16(cpu, SP, cpu->registers.sp - 1);
    writeData(cpu->registers.sp, ((cpu->registers.pc + 2) & 0xFF00) >> 8);
    wr16(cpu, SP, cpu->registers.sp - 1);
    writeData(cpu->registers.sp, ((cpu->registers.pc + 2) & 0x00FF));
    word j = (0x0000 + readData(cpu->registers.pc)) + ((0x0000 + readData(cpu->registers.pc + 1)) << 8);
    wr16(cpu, PC, j);
}

void CPL(CPU* cpu) {
    wr8(cpu, A, (~cpu->registers.a) & 0xFF);
    cpu->registers.f |= 0x60;
    cpu->clock.c += 4;
}

void CCF(CPU* cpu) {
    cpu->registers.f &= 0x9F;
    if ((cpu->registers.f & 0x10) != 0) {
    	cpu->registers.f &= 0xE0;
    }
    else {
    	cpu->registers.f |= 0x10;
    }
    cpu->clock.c += 4;
}

void SCF(CPU* cpu) {
    cpu->registers.f &= 0x9F;
    cpu->registers.f |= 0x10;
    cpu->clock.c += 4;
}

void DAA(CPU* cpu) {
    byte sub = (cpu->registers.f & 0x40) ? 1 : 0;
    byte h = (cpu->registers.f & 0x20) ? 1 : 0;
    byte c = (cpu->registers.f & 0x10) ? 1 : 0;
    int suba = cpu->registers.a;

    if (sub == 1) {
        if (h == 1) {
            cpu->registers.a -= 0x6;
            cpu->registers.a &= 0xFF;
            suba -= 0x6;
            suba &= 0xFF;
        }
        if (c == 1)  {
            cpu->registers.a -= 0x60;
            suba -= 0x60;
        }
    }
    else {
        if (((cpu->registers.a & 0xF) > 9) || h == 1) {
            cpu->registers.a += 0x6;
            suba += 0x6;
        }
        if ((cpu->registers.a > 0x9F) || c == 1) {
            cpu->registers.a += 0x60;
            suba += 0x60;
        }
    }

    if ((suba & 0x100) != 0) c = 1;

    cpu->registers.a &= 0xFF;
    cpu->registers.f &= 0x40;

    if (cpu->registers.a == 0) cpu->registers.f |= 0x80;
    cpu->clock.c += 4;
}

void HALT(CPU* cpu) {
    halt(cpu);
    cpu->clock.c += 4;
}

void DI(CPU* cpu) {
    disableInterrupts(cpu);
    cpu->clock.c += 4;
}

void EI(CPU* cpu) {
    enableInterrupts(cpu);
    cpu->clock.c += 4;
}

void RETI(CPU* cpu) {
    enableInterrupts(cpu);
    RET(cpu);
}

//End Instructions

//Start Opcodes

void x00(CPU* cpu) {cpu->clock.c += 4;}
void x01(CPU* cpu) {LDrrnn(cpu, B, C);}
void x02(CPU* cpu) {LDrrar(cpu, B, C, A);}
void x03(CPU* cpu) {INCrr(cpu, B, C); }
void x04(CPU* cpu) {INCr(cpu, B);}
void x05(CPU* cpu) {DECr(cpu, B);}
void x06(CPU* cpu) {LDrn(cpu, B);}
void x07(CPU* cpu) {
	int out = (rr8(cpu, A) & 0x80) ? 1 : 0;
	out ? wr8(cpu, F, 0x10) : wr8(cpu, F, 0x00);
	wr8(cpu, A, (rr8(cpu, A)<<1) + out);
	cpu->clock.c += 4;
}
void x08(CPU* cpu) {LDnnsp(cpu);}
void x09(CPU* cpu) {ADDrrrr(cpu, H, L, B, C);}
void x0A(CPU* cpu) {LDrrra(cpu, A, B, C);}
void x0B(CPU* cpu) {DECrr(cpu, B, C);}
void x0C(CPU* cpu) {INCr(cpu, C); }
void x0D(CPU* cpu) {DECr(cpu, C);}
void x0E(CPU* cpu) {LDrn(cpu, C);}
void x0F(CPU* cpu) {
	byte out = rr8(cpu, A) & 0x01;
	out ? wr8(cpu, F, 0x10) : wr8(cpu, F, 0x00);
	wr8(cpu, A, (rr8(cpu, A)>>1) | (out*0x80));
	cpu->clock.c += 4;
}

void x10(CPU* cpu) {cpu->registers.pc++; cpu->clock.c += 4;}
void x11(CPU* cpu) {LDrrnn(cpu, D, E);}
void x12(CPU* cpu) {LDrrar(cpu, D, E, A);}
void x13(CPU* cpu) {INCrr(cpu, D, E);}
void x14(CPU* cpu) {INCr(cpu, D);}
void x15(CPU* cpu) {DECr(cpu, D);}
void x16(CPU* cpu) {LDrn(cpu, D);}
void x17(CPU* cpu) {
	int c = (rr8(cpu, F) & 0x10) ? 1 : 0;
	int out = (rr8(cpu, A) & 0x80) ? 1 : 0;
	out ? wr8(cpu, F, 0x10) : wr8(cpu, F, 0x00);
	wr8(cpu, A, (rr8(cpu, A)<<1) + c);
	cpu->clock.c += 4;
}
void x18(CPU* cpu) {JRn(cpu);}
void x19(CPU* cpu) {ADDrrrr(cpu, H, L, D, E);}
void x1A(CPU* cpu) {LDrrra(cpu, A, D, E);}
void x1B(CPU* cpu) {DECrr(cpu, D, E);}
void x1C(CPU* cpu) {INCr(cpu, E);}
void x1D(CPU* cpu) {DECr(cpu, E);}
void x1E(CPU* cpu) {LDrn(cpu, E);}
void x1F(CPU* cpu) {
	int c = (rr8(cpu, F) & 0x10) ? 1 : 0;
	int out = rr8(cpu, A) & 0x01;
	out ? wr8(cpu, F, 0x10) : wr8(cpu, F, 0x00);
	wr8(cpu, A, (rr8(cpu, A)>>1) | (c*0x80));
	cpu->clock.c += 4;
}

void x20(CPU* cpu) {JRccn(cpu, NZ);}
void x21(CPU* cpu) {LDrrnn(cpu, H, L);}
void x22(CPU* cpu) {LDrrar(cpu, H, L, A); INCrr(cpu, H, L); cpu->clock.c -= 8;}
void x23(CPU* cpu) {INCrr(cpu, H, L);}
void x24(CPU* cpu) {INCr(cpu, H);}
void x25(CPU* cpu) {DECr(cpu, H);}
void x26(CPU* cpu) {LDrn(cpu, H);}
void x27(CPU* cpu) {DAA(cpu);}
void x28(CPU* cpu) {JRccn(cpu, Z);}
void x29(CPU* cpu) {ADDrrrr(cpu, H, L, H, L);}
void x2A(CPU* cpu) {LDrrra(cpu, A, H, L); INCrr(cpu, H, L); cpu->clock.c -= 8;}
void x2B(CPU* cpu) {DECrr(cpu, H, L);}
void x2C(CPU* cpu) {INCr(cpu, L);}
void x2D(CPU* cpu) {DECr(cpu, L);}
void x2E(CPU* cpu) {LDrn(cpu, L);}
void x2F(CPU* cpu) {CPL(cpu);}

void x30(CPU* cpu) {JRccn(cpu, NC);}
void x31(CPU* cpu) {LDspnn(cpu);}
void x32(CPU* cpu) {LDrrar(cpu, H, L, A); DECrr(cpu, H, L); cpu->clock.c -= 8;}
void x33(CPU* cpu) {INCsp(cpu);}
void x34(CPU* cpu) {INCrra(cpu, H, L);}
void x35(CPU* cpu) {DECrra(cpu, H, L);}
void x36(CPU* cpu) {LDrran(cpu, H, L);}
void x37(CPU* cpu) {SCF(cpu);}
void x38(CPU* cpu) {JRccn(cpu, C);}
void x39(CPU* cpu) {ADDrrsp(cpu, H, L);}
void x3A(CPU* cpu) {LDrrra(cpu, A, H, L); DECrr(cpu, H, L); cpu->clock.c -= 8;}
void x3B(CPU* cpu) {DECsp(cpu);}
void x3C(CPU* cpu) {INCr(cpu, A);}
void x3D(CPU* cpu) {DECr(cpu, A);}
void x3E(CPU* cpu) {LDrn(cpu, A);}
void x3F(CPU* cpu) {CCF(cpu);}

void x40(CPU* cpu){LDrr(cpu, B, B);}
void x41(CPU* cpu){LDrr(cpu, B, C);}
void x42(CPU* cpu){LDrr(cpu, B, D);}
void x43(CPU* cpu){LDrr(cpu, B, E);}
void x44(CPU* cpu){LDrr(cpu, B, H);}
void x45(CPU* cpu){LDrr(cpu, B, L);}
void x46(CPU* cpu){LDrrra(cpu, B, H, L);}
void x47(CPU* cpu){LDrr(cpu, B, A);}
void x48(CPU* cpu){LDrr(cpu, C, B);}
void x49(CPU* cpu){LDrr(cpu, C, C);}
void x4A(CPU* cpu){LDrr(cpu, C, D);}
void x4B(CPU* cpu){LDrr(cpu, C, E);}
void x4C(CPU* cpu){LDrr(cpu, C, H);}
void x4D(CPU* cpu){LDrr(cpu, C, L);}
void x4E(CPU* cpu){LDrrra(cpu, C, H, L);}
void x4F(CPU* cpu){LDrr(cpu, C, A);}

void x50(CPU* cpu){LDrr(cpu, D, B);}
void x51(CPU* cpu){LDrr(cpu, D, C);}
void x52(CPU* cpu){LDrr(cpu, D, D);}
void x53(CPU* cpu){LDrr(cpu, D, E);}
void x54(CPU* cpu){LDrr(cpu, D, H);}
void x55(CPU* cpu){LDrr(cpu, D, L);}
void x56(CPU* cpu){LDrrra(cpu, D, H, L);}
void x57(CPU* cpu){LDrr(cpu, D, A);}
void x58(CPU* cpu){LDrr(cpu, E, B);}
void x59(CPU* cpu){LDrr(cpu, E, C);}
void x5A(CPU* cpu){LDrr(cpu, E, D);}
void x5B(CPU* cpu){LDrr(cpu, E, E);}
void x5C(CPU* cpu){LDrr(cpu, E, H);}
void x5D(CPU* cpu){LDrr(cpu, E, L);}
void x5E(CPU* cpu){LDrrra(cpu, E, H, L);}
void x5F(CPU* cpu){LDrr(cpu, E, A);}

void x60(CPU* cpu){LDrr(cpu, H, B);}
void x61(CPU* cpu){LDrr(cpu, H, C);}
void x62(CPU* cpu){LDrr(cpu, H, D);}
void x63(CPU* cpu){LDrr(cpu, H, E);}
void x64(CPU* cpu){LDrr(cpu, H, H);}
void x65(CPU* cpu){LDrr(cpu, H, L);}
void x66(CPU* cpu){LDrrra(cpu, H, H, L);}
void x67(CPU* cpu){LDrr(cpu, H, A);}
void x68(CPU* cpu){LDrr(cpu, L, B);}
void x69(CPU* cpu){LDrr(cpu, L, C);}
void x6A(CPU* cpu){LDrr(cpu, L, D);}
void x6B(CPU* cpu){LDrr(cpu, L, E);}
void x6C(CPU* cpu){LDrr(cpu, L, H);}
void x6D(CPU* cpu){LDrr(cpu, L, L);}
void x6E(CPU* cpu){LDrrra(cpu, L, H, L);}
void x6F(CPU* cpu){LDrr(cpu, L, A);}

void x70(CPU* cpu){LDrrar(cpu, H, L, B);}
void x71(CPU* cpu){LDrrar(cpu, H, L, C);}
void x72(CPU* cpu){LDrrar(cpu, H, L, D);}
void x73(CPU* cpu){LDrrar(cpu, H, L, E);}
void x74(CPU* cpu){LDrrar(cpu, H, L, H);}
void x75(CPU* cpu){LDrrar(cpu, H, L, L);}
void x76(CPU* cpu){HALT(cpu);}
void x77(CPU* cpu){LDrrar(cpu, H, L, A);}
void x78(CPU* cpu){LDrr(cpu, A, B);}
void x79(CPU* cpu){LDrr(cpu, A, C);}
void x7A(CPU* cpu){LDrr(cpu, A, D);}
void x7B(CPU* cpu){LDrr(cpu, A, E);}
void x7C(CPU* cpu){LDrr(cpu, A, H);}
void x7D(CPU* cpu){LDrr(cpu, A, L);}
void x7E(CPU* cpu){LDrrra(cpu, A, H, L);}
void x7F(CPU* cpu){LDrr(cpu, A, A);}

void x80(CPU* cpu) {ADDrr(cpu, A, B);}
void x81(CPU* cpu) {ADDrr(cpu, A, C);}
void x82(CPU* cpu) {ADDrr(cpu, A, D);}
void x83(CPU* cpu) {ADDrr(cpu, A, E);}
void x84(CPU* cpu) {ADDrr(cpu, A, H);}
void x85(CPU* cpu) {ADDrr(cpu, A, L);}
void x86(CPU* cpu) {ADDrrra(cpu, A, H, L);}
void x87(CPU* cpu) {ADDrr(cpu, A, A);}
void x88(CPU* cpu) {ADCrr(cpu, A, B);}
void x89(CPU* cpu) {ADCrr(cpu, A, C);}
void x8A(CPU* cpu) {ADCrr(cpu, A, D);}
void x8B(CPU* cpu) {ADCrr(cpu, A, E);}
void x8C(CPU* cpu) {ADCrr(cpu, A, H);}
void x8D(CPU* cpu) {ADCrr(cpu, A, L);}
void x8E(CPU* cpu) {ADCrrra(cpu, A, H, L);}
void x8F(CPU* cpu) {ADCrr(cpu, A, A);}

void x90(CPU* cpu) {SUBr(cpu, B);}
void x91(CPU* cpu) {SUBr(cpu, C);}
void x92(CPU* cpu) {SUBr(cpu, D);}
void x93(CPU* cpu) {SUBr(cpu, E);}
void x94(CPU* cpu) {SUBr(cpu, H);}
void x95(CPU* cpu) {SUBr(cpu, L);}
void x96(CPU* cpu) {SUBrra(cpu, H, L);}
void x97(CPU* cpu) {SUBr(cpu, A);}
void x98(CPU* cpu) {SBCr(cpu, B);}
void x99(CPU* cpu) {SBCr(cpu, C);}
void x9A(CPU* cpu) {SBCr(cpu, D);}
void x9B(CPU* cpu) {SBCr(cpu, E);}
void x9C(CPU* cpu) {SBCr(cpu, H);}
void x9D(CPU* cpu) {SBCr(cpu, L);}
void x9E(CPU* cpu) {SBCrra(cpu, H, L);}
void x9F(CPU* cpu) {SBCr(cpu, A);}

void xA0(CPU* cpu) {ANDr(cpu, B);}
void xA1(CPU* cpu) {ANDr(cpu, C);}
void xA2(CPU* cpu) {ANDr(cpu, D);}
void xA3(CPU* cpu) {ANDr(cpu, E);}
void xA4(CPU* cpu) {ANDr(cpu, H);}
void xA5(CPU* cpu) {ANDr(cpu, L);}
void xA6(CPU* cpu) {ANDrra(cpu, H, L);}
void xA7(CPU* cpu) {XORr(cpu, A);}
void xA8(CPU* cpu) {XORr(cpu, B);}
void xA9(CPU* cpu) {XORr(cpu, C);}
void xAA(CPU* cpu) {XORr(cpu, D);}
void xAB(CPU* cpu) {XORr(cpu, E);}
void xAC(CPU* cpu) {XORr(cpu, H);}
void xAD(CPU* cpu) {XORr(cpu, L);}
void xAE(CPU* cpu) {XORrra(cpu, H, L);}
void xAF(CPU* cpu) {XORr(cpu, A);}

void xB0(CPU* cpu) {ORr(cpu, B);}
void xB1(CPU* cpu) {ORr(cpu, C);}
void xB2(CPU* cpu) {ORr(cpu, D);}
void xB3(CPU* cpu) {ORr(cpu, E);}
void xB4(CPU* cpu) {ORr(cpu, H);}
void xB5(CPU* cpu) {ORr(cpu, L);}
void xB6(CPU* cpu) {ORrra(cpu, H, L);}
void xB7(CPU* cpu) {ORr(cpu, A);}
void xB8(CPU* cpu) {CPr(cpu, B);}
void xB9(CPU* cpu) {CPr(cpu, C);}
void xBA(CPU* cpu) {CPr(cpu, D);}
void xBB(CPU* cpu) {CPr(cpu, E);}
void xBC(CPU* cpu) {CPr(cpu, H);}
void xBD(CPU* cpu) {CPr(cpu, L);}
void xBE(CPU* cpu) {CPrra(cpu, H, L);}
void xBF(CPU* cpu) {CPr(cpu, A);}

void xC0(CPU* cpu) {RETcc(cpu, NZ);}
void xC1(CPU* cpu) {POPrr(cpu, B, C);}
void xC2(CPU* cpu) {JPccnn(cpu, NZ);}
void xC3(CPU* cpu) {JPnn(cpu);}
void xC4(CPU* cpu) {CALLccnn(cpu, NZ);}
void xC5(CPU* cpu) {PUSHrr(cpu, B, C);}
void xC6(CPU* cpu) {ADDrn(cpu, A);}
void xC7(CPU* cpu) {RSTn(cpu, 0x00);}
void xC8(CPU* cpu) {RETcc(cpu, Z);}
void xC9(CPU* cpu) {RET(cpu);}
void xCA(CPU* cpu) {JPccnn(cpu, Z);}
//void xCB(CPU* cpu) {CB(cpu);}
void xCC(CPU* cpu) {CALLccnn(cpu, Z);}
void xCD(CPU* cpu) {CALLnn(cpu);}
void xCE(CPU* cpu) {ADCrn(cpu, A);}
void xCF(CPU* cpu) {RSTn(cpu, 0x08);}

void xD0(CPU* cpu) {RETcc(cpu, NC);}
void xD1(CPU* cpu) {POPrr(cpu, D, E);}
void xD2(CPU* cpu) {JPccnn(cpu, NC);}
//xD3
void xD4(CPU* cpu) {CALLccnn(cpu, NC);}
void xD5(CPU* cpu) {PUSHrr(cpu, D, E);}
void xD6(CPU* cpu) {SUBn(cpu);}
void xD7(CPU* cpu) {RSTn(cpu, 0x10);}
void xD8(CPU* cpu) {RETcc(cpu, C);}
void xD9(CPU* cpu) {RETI(cpu);}
void xDA(CPU* cpu) {JPccnn(cpu, C);}
//xDB
void xDC(CPU* cpu) {CALLccnn(cpu, C);}
//xDD
void xDE(CPU* cpu) {SBCn(cpu);}
void xDF(CPU* cpu) {RSTn(cpu, 0x18);}

void xE0(CPU* cpu) {LDHnar(cpu, A);}
void xE1(CPU* cpu) {POPrr(cpu, H, L);}
void xE2(CPU* cpu) {LDrar(cpu, C, A);}
//xE3
//xE4
void xE5(CPU* cpu) {PUSHrr(cpu, H, L);}
void xE6(CPU* cpu) {ANDn(cpu);}
void xE7(CPU* cpu) {RSTn(cpu, 0x20);}
void xE8(CPU* cpu) {ADDspn(cpu);}
void xE9(CPU* cpu) {JPrr(cpu, H, L);}
void xEA(CPU* cpu) {LDnnar(cpu, A);}
//xEB
//xEC
//xED
void xEE(CPU* cpu) {XORn(cpu);}
void xEF(CPU* cpu) {RSTn(cpu, 0x28);}

void xF0(CPU* cpu) {LDHrna(cpu, A);}
void xF1(CPU* cpu) {POPrr(cpu, A, F);}
void xF2(CPU* cpu) {LDrra(cpu, A, C);}
void xF3(CPU* cpu) {DI(cpu);}
//xF4
void xF5(CPU* cpu) {PUSHrr(cpu, A, F);}
void xF6(CPU* cpu) {ORn(cpu);}
void xF7(CPU* cpu) {RSTn(cpu, 0x30);}
void xF8(CPU* cpu) {LDrrspn(cpu, H, L);}
void xF9(CPU* cpu) {LDsprr(cpu, H, L);}
void xFA(CPU* cpu) {LDrnna(cpu, A);}
void xFB(CPU* cpu) {EI(cpu);}
//xFC
//xFD
void xFE(CPU* cpu) {CPn(cpu);}
void xFF(CPU* cpu) {RSTn(cpu, 0x38);}

void initOpcodes(){
	for(int i = 0; i < 256; i++) {
		opcodeArray[i] = NULL;
	}
	opcodeArray[0x00] = &x00;
	opcodeArray[0x01] = &x01;
	opcodeArray[0x02] = &x02;
	opcodeArray[0x03] = &x03;
	opcodeArray[0x04] = &x04;
	opcodeArray[0x05] = &x05;
	opcodeArray[0x06] = &x06;
	opcodeArray[0x07] = &x07;
	opcodeArray[0x08] = &x08;
	opcodeArray[0x09] = &x09;
	opcodeArray[0x0A] = &x0A;
	opcodeArray[0x0B] = &x0B;
	opcodeArray[0x0C] = &x0C;
	opcodeArray[0x0D] = &x0D;
	opcodeArray[0x0E] = &x0E;
	opcodeArray[0x0F] = &x0F;
	opcodeArray[0x10] = &x10;
	opcodeArray[0x11] = &x11;
	opcodeArray[0x12] = &x12;
	opcodeArray[0x13] = &x13;
	opcodeArray[0x14] = &x14;
	opcodeArray[0x15] = &x15;
	opcodeArray[0x16] = &x16;
	opcodeArray[0x17] = &x17;
	opcodeArray[0x18] = &x18;
	opcodeArray[0x19] = &x19;
	opcodeArray[0x1A] = &x1A;
	opcodeArray[0x1B] = &x1B;
	opcodeArray[0x1C] = &x1C;
	opcodeArray[0x1D] = &x1D;
	opcodeArray[0x1E] = &x1E;
	opcodeArray[0x1F] = &x1F;
	opcodeArray[0x20] = &x20;
	opcodeArray[0x21] = &x21;
	opcodeArray[0x22] = &x22;
	opcodeArray[0x23] = &x23;
	opcodeArray[0x24] = &x24;
	opcodeArray[0x25] = &x25;
	opcodeArray[0x26] = &x26;
	opcodeArray[0x27] = &x27;
	opcodeArray[0x28] = &x28;
	opcodeArray[0x29] = &x29;
	opcodeArray[0x2A] = &x2A;
	opcodeArray[0x2B] = &x2B;
	opcodeArray[0x2C] = &x2C;
	opcodeArray[0x2D] = &x2D;
	opcodeArray[0x2E] = &x2E;
	opcodeArray[0x2F] = &x2F;
	opcodeArray[0x30] = &x30;
	opcodeArray[0x31] = &x31;
	opcodeArray[0x32] = &x32;
	opcodeArray[0x33] = &x33;
	opcodeArray[0x34] = &x34;
	opcodeArray[0x35] = &x35;
	opcodeArray[0x36] = &x36;
	opcodeArray[0x37] = &x37;
	opcodeArray[0x38] = &x38;
	opcodeArray[0x39] = &x39;
	opcodeArray[0x3A] = &x3A;
	opcodeArray[0x3B] = &x3B;
	opcodeArray[0x3C] = &x3C;
	opcodeArray[0x3D] = &x3D;
	opcodeArray[0x3E] = &x3E;
	opcodeArray[0x3F] = &x3F;
	opcodeArray[0x40] = &x40;
	opcodeArray[0x41] = &x41;
	opcodeArray[0x42] = &x42;
	opcodeArray[0x43] = &x43;
	opcodeArray[0x44] = &x44;
	opcodeArray[0x45] = &x45;
	opcodeArray[0x46] = &x46;
	opcodeArray[0x47] = &x47;
	opcodeArray[0x48] = &x48;
	opcodeArray[0x49] = &x49;
	opcodeArray[0x4A] = &x4A;
	opcodeArray[0x4B] = &x4B;
	opcodeArray[0x4C] = &x4C;
	opcodeArray[0x4D] = &x4D;
	opcodeArray[0x4E] = &x4E;
	opcodeArray[0x4F] = &x4F;
	opcodeArray[0x50] = &x50;
	opcodeArray[0x51] = &x51;
	opcodeArray[0x52] = &x52;
	opcodeArray[0x53] = &x53;
	opcodeArray[0x54] = &x54;
	opcodeArray[0x55] = &x55;
	opcodeArray[0x56] = &x56;
	opcodeArray[0x57] = &x57;
	opcodeArray[0x58] = &x58;
	opcodeArray[0x59] = &x59;
	opcodeArray[0x5A] = &x5A;
	opcodeArray[0x5B] = &x5B;
	opcodeArray[0x5C] = &x5C;
	opcodeArray[0x5D] = &x5D;
	opcodeArray[0x5E] = &x5E;
	opcodeArray[0x5F] = &x5F;
	opcodeArray[0x60] = &x60;
	opcodeArray[0x61] = &x61;
	opcodeArray[0x62] = &x62;
	opcodeArray[0x63] = &x63;
	opcodeArray[0x64] = &x64;
	opcodeArray[0x65] = &x65;
	opcodeArray[0x66] = &x66;
	opcodeArray[0x67] = &x67;
	opcodeArray[0x68] = &x68;
	opcodeArray[0x69] = &x69;
	opcodeArray[0x6A] = &x6A;
	opcodeArray[0x6B] = &x6B;
	opcodeArray[0x6C] = &x6C;
	opcodeArray[0x6D] = &x6D;
	opcodeArray[0x6E] = &x6E;
	opcodeArray[0x6F] = &x6F;
	opcodeArray[0x70] = &x70;
	opcodeArray[0x71] = &x71;
	opcodeArray[0x72] = &x72;
	opcodeArray[0x73] = &x73;
	opcodeArray[0x74] = &x74;
	opcodeArray[0x75] = &x75;
	opcodeArray[0x76] = &x76;
	opcodeArray[0x77] = &x77;
	opcodeArray[0x78] = &x78;
	opcodeArray[0x79] = &x79;
	opcodeArray[0x7A] = &x7A;
	opcodeArray[0x7B] = &x7B;
	opcodeArray[0x7C] = &x7C;
	opcodeArray[0x7D] = &x7D;
	opcodeArray[0x7E] = &x7E;
	opcodeArray[0x7F] = &x7F;
	opcodeArray[0x80] = &x80;
	opcodeArray[0x81] = &x81;
	opcodeArray[0x82] = &x82;
	opcodeArray[0x83] = &x83;
	opcodeArray[0x84] = &x84;
	opcodeArray[0x85] = &x85;
	opcodeArray[0x86] = &x86;
	opcodeArray[0x87] = &x87;
	opcodeArray[0x88] = &x88;
	opcodeArray[0x89] = &x89;
	opcodeArray[0x8A] = &x8A;
	opcodeArray[0x8B] = &x8B;
	opcodeArray[0x8C] = &x8C;
	opcodeArray[0x8D] = &x8D;
	opcodeArray[0x8E] = &x8E;
	opcodeArray[0x8F] = &x8F;
	opcodeArray[0x90] = &x90;
	opcodeArray[0x91] = &x91;
	opcodeArray[0x92] = &x92;
	opcodeArray[0x93] = &x93;
	opcodeArray[0x94] = &x94;
	opcodeArray[0x95] = &x95;
	opcodeArray[0x96] = &x96;
	opcodeArray[0x97] = &x97;
	opcodeArray[0x98] = &x98;
	opcodeArray[0x99] = &x99;
	opcodeArray[0x9A] = &x9A;
	opcodeArray[0x9B] = &x9B;
	opcodeArray[0x9C] = &x9C;
	opcodeArray[0x9D] = &x9D;
	opcodeArray[0x9E] = &x9E;
	opcodeArray[0x9F] = &x9F;
	opcodeArray[0xA0] = &xA0;
	opcodeArray[0xA1] = &xA1;
	opcodeArray[0xA2] = &xA2;
	opcodeArray[0xA3] = &xA3;
	opcodeArray[0xA4] = &xA4;
	opcodeArray[0xA5] = &xA5;
	opcodeArray[0xA6] = &xA6;
	opcodeArray[0xA7] = &xA7;
	opcodeArray[0xA8] = &xA8;
	opcodeArray[0xA9] = &xA9;
	opcodeArray[0xAA] = &xAA;
	opcodeArray[0xAB] = &xAB;
	opcodeArray[0xAC] = &xAC;
	opcodeArray[0xAD] = &xAD;
	opcodeArray[0xAE] = &xAE;
	opcodeArray[0xAF] = &xAF;
	opcodeArray[0xB0] = &xB0;
	opcodeArray[0xB1] = &xB1;
	opcodeArray[0xB2] = &xB2;
	opcodeArray[0xB3] = &xB3;
	opcodeArray[0xB4] = &xB4;
	opcodeArray[0xB5] = &xB5;
	opcodeArray[0xB6] = &xB6;
	opcodeArray[0xB7] = &xB7;
	opcodeArray[0xB8] = &xB8;
	opcodeArray[0xB9] = &xB9;
	opcodeArray[0xBA] = &xBA;
	opcodeArray[0xBB] = &xBB;
	opcodeArray[0xBC] = &xBC;
	opcodeArray[0xBD] = &xBD;
	opcodeArray[0xBE] = &xBE;
	opcodeArray[0xBF] = &xBF;
	opcodeArray[0xC0] = &xC0;
	opcodeArray[0xC1] = &xC1;
	opcodeArray[0xC2] = &xC2;
	opcodeArray[0xC3] = &xC3;
	opcodeArray[0xC4] = &xC4;
	opcodeArray[0xC5] = &xC5;
	opcodeArray[0xC6] = &xC6;
	opcodeArray[0xC7] = &xC7;
	opcodeArray[0xC8] = &xC8;
	opcodeArray[0xC9] = &xC9;
	opcodeArray[0xCA] = &xCA;
//	opcodeArray[0xCB] = &xCB;
	opcodeArray[0xCC] = &xCC;
	opcodeArray[0xCD] = &xCD;
	opcodeArray[0xCE] = &xCE;
	opcodeArray[0xCF] = &xCF;
	opcodeArray[0xD0] = &xD0;
	opcodeArray[0xD1] = &xD1;
	opcodeArray[0xD2] = &xD2;
//	opcodeArray[0xD3] = &xD3;
	opcodeArray[0xD4] = &xD4;
	opcodeArray[0xD5] = &xD5;
	opcodeArray[0xD6] = &xD6;
	opcodeArray[0xD7] = &xD7;
	opcodeArray[0xD8] = &xD8;
	opcodeArray[0xD9] = &xD9;
	opcodeArray[0xDA] = &xDA;
//	opcodeArray[0xDB] = &xDB;
	opcodeArray[0xDC] = &xDC;
//	opcodeArray[0xDD] = &xDD;
	opcodeArray[0xDE] = &xDE;
	opcodeArray[0xDF] = &xDF;
	opcodeArray[0xE0] = &xE0;
	opcodeArray[0xE1] = &xE1;
	opcodeArray[0xE2] = &xE2;
//	opcodeArray[0xE3] = &xE3;
//	opcodeArray[0xE4] = &xE4;
	opcodeArray[0xE5] = &xE5;
	opcodeArray[0xE6] = &xE6;
	opcodeArray[0xE7] = &xE7;
	opcodeArray[0xE8] = &xE8;
	opcodeArray[0xE9] = &xE9;
	opcodeArray[0xEA] = &xEA;
//	opcodeArray[0xEB] = &xEB;
//	opcodeArray[0xEC] = &xEC;
//	opcodeArray[0xED] = &xED;
	opcodeArray[0xEE] = &xEE;
	opcodeArray[0xEF] = &xEF;
	opcodeArray[0xF0] = &xF0;
	opcodeArray[0xF1] = &xF1;
	opcodeArray[0xF2] = &xF2;
	opcodeArray[0xF3] = &xF3;
//	opcodeArray[0xF4] = &xF4;
	opcodeArray[0xF5] = &xF5;
	opcodeArray[0xF6] = &xF6;
	opcodeArray[0xF7] = &xF7;
	opcodeArray[0xF8] = &xF8;
	opcodeArray[0xF9] = &xF9;
	opcodeArray[0xFA] = &xFA;
	opcodeArray[0xFB] = &xFB;
//	opcodeArray[0xFC] = &xFC;
//	opcodeArray[0xFD] = &xFD;
	opcodeArray[0xFE] = &xFE;
	opcodeArray[0xFF] = &xFF;

}


//End Opcodes

int main() {

	printf("Starting\n");
	initOpcodes();
	CPU cpu;
	backup = &cpu;
	init_CPU(&cpu);
	GPU gpu;
	initGPU(&gpu);
	cpu.gpu = &gpu;
	getGameName();
	run(&cpu);
	return 0;
}

/*
 *
 *
 *printf("Hi\n");
	for(int y = 0; y < 144; y++){
		for(int x = 0; x < 160; x ++) {
			setPixel(x, y, 0);
		}
	}
	setPixel(0, 8, 3);
			setPixel(1, 8, 3);
			setPixel(2, 8, 3);
			setPixel(3, 8, 3);
			setPixel(4, 8, 3);
			setPixel(5, 8, 3);
			setPixel(6, 8, 3);
			setPixel(7, 8, 3);

			setPixel(0, 9, 3);
			setPixel(1, 9, 2);
			setPixel(2, 9, 2);
			setPixel(3, 9, 2);
			setPixel(4, 9, 2);
			setPixel(5, 9, 2);
			setPixel(6, 9, 2);
			setPixel(7, 9, 3);

			setPixel(0, 10, 3);
			setPixel(1, 10, 2);
			setPixel(2, 10, 2);
			setPixel(3, 10, 2);
			setPixel(4, 10, 2);
			setPixel(5, 10, 2);
			setPixel(6, 10, 2);
			setPixel(7, 10, 3);

			setPixel(0, 11, 3);
			setPixel(1, 11, 2);
			setPixel(2, 11, 2);
			setPixel(3, 11, 2);
			setPixel(4, 11, 2);
			setPixel(5, 11, 2);
			setPixel(6, 11, 2);
			setPixel(7, 11, 3);

			setPixel(0, 12, 3);
			setPixel(1, 12, 2);
			setPixel(2, 12, 2);
			setPixel(3, 12, 2);
			setPixel(4, 12, 2);
			setPixel(5, 12, 2);
			setPixel(6, 12, 2);
			setPixel(7, 12, 3);

			setPixel(0, 13, 3);
			setPixel(1, 13, 2);
			setPixel(2, 13, 2);
			setPixel(3, 13, 2);
			setPixel(4, 13, 2);
			setPixel(5, 13, 2);
			setPixel(6, 13, 2);
			setPixel(7, 13, 3);

			setPixel(0, 14, 3);
			setPixel(1, 14, 2);
			setPixel(2, 14, 2);
			setPixel(3, 14, 2);
			setPixel(4, 14, 2);
			setPixel(5, 14, 2);
			setPixel(6, 14, 2);
			setPixel(7, 14, 3);

			setPixel(0, 15, 3);
			setPixel(1, 15, 3);
			setPixel(2, 15, 3);
			setPixel(3, 15, 3);
			setPixel(4, 15, 3);
			setPixel(5, 15, 3);
			setPixel(6, 15, 3);
			setPixel(7, 15, 3);
	for(int y = 0; y < 144; y++){
		for(int x = 0; x < 160; x ++) {
			int c = buffer[160 * y + x];
			if(c == 0) {
				printf("0");
			}else {
				printf("1");
			}
		}
		printf("\n");
	}
	render();
 *
 *
 */



