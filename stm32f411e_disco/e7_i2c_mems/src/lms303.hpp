/** LMS303 example driver for EP 
 *   Lucjan Bryndza BOFF
*/
#include <cstddef>

namespace periph::drivers {
    class i2c_master;
}

namespace app {

    class lms303 {
    public:
        explicit lms303(periph::drivers::i2c_master& );
        lms303(lms303&) = delete;
        lms303& operator=(lms303&) = delete;
        int configure();
        int get_accel(int& x, int& y, int& z);
        int get_orientation(int& x, int& y, int& z);
    private:
        int write_reg(int addr, int reg,  unsigned char value);
        int read_reg(int addr, int reg, unsigned char& value);
    private:
        periph::drivers::i2c_master& m_i2c;
    };
}
