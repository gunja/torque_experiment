CFLAGS += $(pkg-config --cflags soem)
CFLAGS += -I/opt/ros/melodic/include/soem
CXXFLAGS += -I/opt/ros/melodic/include/soem

LDFLAGS += $(shell pkg-config --libs soem)


.PHONY: all
all:    app

app: app.o support.o

set_pdo: set_pdo.o support.o

set_pdos: set_pdos.o
	$(CXX)  -o $@ $< $(LDFLAGS)

.PHONY: clean
clean:
	rm -rf *.o
