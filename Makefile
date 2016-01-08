########################################################################
##
## Prácticas Visión por computador, curso 2015-2016
## Basado en el makefile de Carlos Ureña, Oct-2014, para las prácticas de
## Informática Gráfica.
##
## archivo 'make' para compilar, enlazar y ejecutar
## invocar con 'make'
## genera archivo ejecutable 'prac', usando todos los .cpp/.cc/.c presentes
##
########################################################################

.SUFFIXES:
.PHONY: start, exec, all, compile, clean, tar

INC					:= ./inc
BIN					:= ./bin
OBJ					:= ./obj
SRC					:= ./src

NUM					:= 1
ZIP_DIR				:= ../GarciaMontoro_Alejandro-$(NUM)

target_name         := $(BIN)/prac
opt_dbg_flag        := -g
exit_first          := -Wfatal-errors
warn_all            := -Wall
units_ext           := $(wildcard $(SRC)/*.cpp)
headers             := $(wildcard $(INC)/*.hpp)
comp_version		:= -std=c++11

gl_libs             := -lopencv_shape -lopencv_stitching -lopencv_objdetect -lopencv_superres -lopencv_videostab -lopencv_calib3d -lopencv_features2d -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_video -lopencv_photo -lopencv_ml -lopencv_imgproc -lopencv_flann -lopencv_core #$(shell pkg-config --libs opencv)
other_ld_libs       :=


units               := $(basename $(units_ext))
objs                := $(addprefix $(OBJ)/, $(addsuffix .o, $(notdir $(units))))
c_flags             := -I $(INC) -I $(SRC) $(opt_dbg_flag) $(exit_first) $(warn_all) $(comp_version)
ld_libs             := $(gl_libs) $(other_ld_libs)

start:
	@make --no-print-directory exec

exec: $(target_name)
	@echo "ejecutando " $(target_name) " ...."
	./$(target_name)

all:
	echo $(objs)
	make clean
	make compile

compile: $(target_name)
	@echo "compilando fuentes: " $(units_ext)
	@make --no-print-directory $(target_name)

$(target_name) : $(objs) | $(BIN)
	@echo `tput bold`---------------------------------------------------------------
	@echo "Enlazando      :" $(target_name)
	@echo "Unidades(ext)  :" $(units_ext)
	@echo "Objetos        :" $(objs)
	@tput sgr0
	g++ -o $(target_name) $(objs) $(ld_libs)
	@echo ---------------------------------------------------------------


$(OBJ)/%.o: $(SRC)/%.cpp $(headers) | $(OBJ)
	@echo `tput bold`---------------------------------------------------------------
	@echo Compilando: $(notdir $<)
	@tput sgr0
	@g++ $(c_flags) -c $< -o $@

# Creation of binary directory
$(BIN):
	mkdir -p $(BIN)

# Creation of objects directory
$(OBJ):
	mkdir -p $(OBJ)

clean:
	rm -f $(OBJ)/*.o $(target_name)

zip:
	@echo `tput bold`---------------------------------------------------------------
	@echo "Arranging the right files for you..."
	@tput sgr0
	@mkdir $(ZIP_DIR)
	@cp -r $(INC) $(ZIP_DIR)
	@cp -r $(SRC) $(ZIP_DIR)
	@cp Informes/I_$(NUM).pdf $(ZIP_DIR)/Informe_$(NUM).pdf
	@echo `tput bold`---------------------------------------------------------------
	@echo "Zipping everything..."
	@tput sgr0
	@zip -r $(ZIP_DIR).zip $(ZIP_DIR)
	@rm -r $(ZIP_DIR)
	@echo `tput bold`---------------------------------------------------------------
	@echo "Done! Do not forget to send the zip to the teacher :)"
	@tput sgr0
