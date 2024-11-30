/*
Student No.: 111550076
Student Name: 楊子賝
Email: zichen55.cs11@mycu.edu.tw
SE tag: xnxcxtxuxoxsx
Statement: I am fully aware that this program is not
supposed to be posted to a public server, such as a
public GitHub repository or a public web page.
*/
#include <fstream>
#include <iostream>
#include <iomanip> 
#include <pthread.h>
#include <queue>
#include <semaphore.h>
#include <sys/time.h>

using namespace std;

const int MAX_N = 1000005;

int n;
int arr[MAX_N];
queue<int> jobs;
sem_t mutex, finish;
int bound[16][2];
bool done[16];

void read() {
    ifstream fin("input.txt");
    fin >> n;
    for (int i = 0; i < n; i++)
        fin >> arr[i];
    fin.close();
}

void output(int idx) {
    string path = "output_" + to_string(idx) + ".txt";
    ofstream fout(path);
    for(int i = 0; i < n; ++i) {
        fout << arr[i] << " ";
    }
    fout.close();
}

void bubblesort(int l, int r) {
    for(int i = l; i < r; ++i) { 
        for(int j = i + 1; j < r; ++j) {
            if(arr[i] > arr[j]) swap(arr[i], arr[j]);
        }
    }
}

void merge(int begin, int mid, int end) {
    int size = end - begin;
    int *tmp = new int[size];
    int l = begin, r = mid, idx = 0;
    
    while (l < mid && r < end) {
        if (arr[l] < arr[r])
            tmp[idx++] = arr[l++];
        else
            tmp[idx++] = arr[r++];
    }
    while (l < mid)
        tmp[idx++] = arr[l++];
    while (r < end)
        tmp[idx++] = arr[r++];
    
    for (int i = 0; i < idx; i++)
        arr[begin + i] = tmp[i];
    
    delete[] tmp;
}

void *solve(void *) {
    while (1) {
        sem_wait(&mutex);
        if (jobs.empty()) {
            sem_post(&mutex);
            sem_post(&finish);
            return nullptr;
        }
        int cur = jobs.front();
        jobs.pop();
        sem_post(&mutex);
        
        if (cur > 7) {
            bubblesort(bound[cur][0], bound[cur][1]);
        } else {
            merge(bound[cur][0], bound[cur * 2][1], bound[cur][1]);
        }
        
        sem_wait(&mutex);
        done[cur] = true;
        if ((cur % 2 && done[cur - 1]) || (cur % 2 == 0 && done[cur + 1])) {
            jobs.push(cur >> 1);
        }
        sem_post(&mutex);
    }
}

int main() {
    pthread_t task[2];
    struct timeval start, end;
    
    int T = 2;
    read();
    for(int i = 15; i >= 0; --i) {
        done[i] = false;
        if(i > 7){
            bound[i][0] = (i - 8) * (n / 8);
            if(i == 15) bound[i][1] = n;
            else bound[i][1] =  bound[i][0] + n / 8;
        }
        else{
            bound[i][0] = bound[i * 2][0];
            bound[i][1] = bound[i * 2 + 1][1];
        }
    }

    while(!jobs.empty()) jobs.pop();
    
    gettimeofday(&start, NULL);
    sem_init(&finish, 0, 0);
    sem_init(&mutex, 0, 1);

    for(int i = 8; i <= 15; ++i) {
        jobs.push(i);
    }

    for(int i = 0; i < T; ++i) {
        pthread_create(&task[i], NULL, solve, NULL);
    }
    for(int i = 0; i < T; ++i) {
        sem_wait(&finish); 
    }

    gettimeofday(&end, NULL);
    output(T);
    
    for(int i = 0; i < T; ++i) {
        pthread_join(task[i], NULL);
    }
    
    sem_destroy(&mutex);
    sem_destroy(&finish);
    
    double time_taken = (end.tv_sec - start.tv_sec) * 1e3 + (end.tv_usec - start.tv_usec) * 1e-3;
    cout << "worker thread #" << T << ", elapsed " << fixed << setprecision(6) << time_taken << " ms\n";

}
