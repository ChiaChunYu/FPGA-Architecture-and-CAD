CXX := g++
CXXFLAGS :=  -std=c++17 -O3 -DNDEBUG -march=native -flto 

BIN_DIR := ../bin
OUT_DIR := ../output
SRC := main.cpp \
		    parser/parser.cpp \
       	    design/design.cpp \
       		writer/writer.cpp \
       		placer/placer.cpp \

CHECKER := ../checker 

TARGET := $(BIN_DIR)/placer

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRC)
	@mkdir -p $(BIN_DIR)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

clean:
	rm -f $(TARGET)

test_alu4: $(TARGET)
	@echo "========================================"
	@echo "[TEST] Starting test alu4"
	@echo "========================================"
	@mkdir -p $(OUT_DIR)
	$(TARGET)  ../testcases/alu4.txt $(OUT_DIR)/alu4.txt
	@$(CHECKER)  ../testcases/alu4.txt $(OUT_DIR)/alu4.txt
	@echo ""

test_apex4: $(TARGET)
	@echo "========================================"
	@echo "[TEST] Starting test apex4"
	@echo "========================================"
	@mkdir -p $(OUT_DIR)
	$(TARGET)  ../testcases/apex4.txt $(OUT_DIR)/apex4.txt
	@$(CHECKER)  ../testcases/apex4.txt $(OUT_DIR)/apex4.txt
	@echo ""


test_clma_2: $(TARGET)
	@echo "========================================"
	@echo "[TEST] Starting test clma_2"
	@echo "========================================"
	@mkdir -p $(OUT_DIR)
	$(TARGET)  ../testcases/clma_2.txt $(OUT_DIR)/clma_2.txt
	@$(CHECKER)  ../testcases/clma_2.txt $(OUT_DIR)/clma_2.txt
	@echo ""


test_elliptic: $(TARGET)
	@echo "========================================"
	@echo "[TEST] Starting test elliptic"
	@echo "========================================"
	@mkdir -p $(OUT_DIR)
	$(TARGET)  ../testcases/elliptic.txt $(OUT_DIR)/elliptic.txt
	@$(CHECKER)  ../testcases/elliptic.txt $(OUT_DIR)/elliptic.txt
	@echo ""


test_pdc: $(TARGET)
	@echo "========================================"
	@echo "[TEST] Starting test pdc"
	@echo "========================================"
	@mkdir -p $(OUT_DIR)
	$(TARGET)  ../testcases/pdc.txt $(OUT_DIR)/pdc.txt
	@$(CHECKER)  ../testcases/pdc.txt $(OUT_DIR)/pdc.txt
	@echo ""

test_pdc_20000: $(TARGET)
	@echo "========================================"
	@echo "[TEST] Starting test pdc_20000"
	@echo "========================================"
	@mkdir -p $(OUT_DIR)
	$(TARGET)  ../testcases/pdc_20000.txt $(OUT_DIR)/pdc_20000.txt
	@$(CHECKER)  ../testcases/pdc_20000.txt $(OUT_DIR)/pdc_20000.txt
	@echo ""


test : test_alu4 test_apex4 test_clma_2 test_elliptic test_pdc test_pdc_20000