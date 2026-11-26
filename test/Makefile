# Host unit tests for the pure modules (REQ-TST-01). `make -C test`
CC      ?= cc
CORE    := ../Challenge_TestHC/Core
SRC     := $(CORE)/Src
CFLAGS  := -std=gnu11 -Wall -Wextra -Wdouble-promotion -Wconversion -Wshadow -Werror -g -O1 \
           -fsanitize=address,undefined -fno-sanitize-recover=undefined -D__MAIN_H -include stubs/host_types.h -I$(CORE)/Inc -I.
PURE    := par prof whl drv fmt proto console att ekf imu_yaw gnd fwd tlm_pure
OBJ     := $(addprefix $(SRC)/,$(addsuffix .c,$(PURE))) sim_plant.c test_main.c \
           t_crc.c t_fmt.c t_par.c t_prof.c t_whl_law.c t_drv_move.c t_con.c t_sched.c \
           t_att.c t_cobs.c t_proto_msgs.c t_tlm.c t_ekf_static.c t_ekf_slip.c t_ekf_gate.c t_ekf_sf_q.c t_ekf_jacobians.c \
           t_ekf_backshift.c t_ekf_line.c t_ekf_pred.c t_imu_yaw.c t_gnd.c t_fwd.c

LINK    := $(addprefix $(SRC)/,tlm.c tlm_pure.c proto.c par.c fmt.c) t_tlm_link.c

test: t_all t_link
	./t_all
	./t_link

t_all: $(OBJ) $(wildcard $(CORE)/Inc/*.h) $(wildcard *.h)
	$(CC) $(CFLAGS) $(OBJ) -lm -o $@

t_link: $(LINK) $(wildcard $(CORE)/Inc/*.h) $(wildcard *.h)
	$(CC) $(CFLAGS) $(LINK) -lm -o $@

clean:
	rm -rf t_all t_all.dSYM t_link t_link.dSYM

.PHONY: test clean
