# Host unit tests for the pure modules (REQ-TST-01). `make -C test`
CC      ?= cc
CORE    := ../Challenge_TestHC/Core
SRC     := $(CORE)/Src
CFLAGS  := -std=gnu11 -Wall -Wextra -Wdouble-promotion -Wconversion -Wshadow -Werror -g -O1 \
           -fsanitize=address,undefined -fno-sanitize-recover=undefined -D__MAIN_H -include stubs/host_types.h -I$(CORE)/Inc -I.
PURE    := par prof whl drv fmt proto con
OBJ     := $(addprefix $(SRC)/,$(addsuffix .c,$(PURE))) sim_plant.c test_main.c \
           t_crc.c t_fmt.c t_par.c t_prof.c t_whl_law.c t_drv_move.c t_con.c

test: t_all
	./t_all

t_all: $(OBJ) $(wildcard $(CORE)/Inc/*.h) $(wildcard *.h)
	$(CC) $(CFLAGS) $(OBJ) -lm -o $@

clean:
	rm -rf t_all t_all.dSYM

.PHONY: test clean
