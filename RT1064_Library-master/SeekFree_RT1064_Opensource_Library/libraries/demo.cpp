 #include <iostream>
using namespace std;

class Counter {
private:
    int count;
public:
    // Default constructor
    Counter() : count(0) {}
    // Constructor with parameter
    Counter(int c) : count(c) {}
    // Getter function for count
    int get_count() const { return count; }
    // Prefix increment operator
    Counter operator ++ () {
        ++count;
        return Counter(count);
    }
    // Postfix increment operator
    Counter operator ++ (int) {
        return Counter(count++);
    }
};

int main() {
    Counter c1, c2;
    cout << "c1 = " << c1.get_count() << endl;
    cout << "c2 = " << c2.get_count() << endl;
    ++c1;
    c2++;
    cout << "c1 = " << c1.get_count() << endl;
    cout << "c2 = " << c2.get_count() << endl;
    return 0;
}