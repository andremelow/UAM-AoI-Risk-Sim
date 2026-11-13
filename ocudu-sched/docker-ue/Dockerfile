# srsUE (srsRAN 4G, 5G SA capable) used as the real over-the-air / ZMQ UE against the OCUDU gNB.
FROM ocudu-dev:24.04
RUN apt-get update && apt-get install -y --no-install-recommends libconfig++-dev libboost-program-options-dev \
    && rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 https://github.com/srsran/srsRAN_4G.git /opt/srsRAN_4G \
    && cmake -S /opt/srsRAN_4G -B /opt/srsRAN_4G/build -G Ninja -DCMAKE_BUILD_TYPE=Release \
       -DENABLE_WERROR=OFF -DENABLE_SRSENB=OFF -DENABLE_SRSEPC=OFF -DENABLE_GUI=OFF \
       -DCMAKE_CXX_FLAGS="-Wno-error" -DCMAKE_C_FLAGS="-Wno-error" \
    && ninja -C /opt/srsRAN_4G/build \
    && cmake --install /opt/srsRAN_4G/build && ldconfig \
    && mkdir -p /etc/srsran && cp /opt/srsRAN_4G/srsue/ue.conf.example /etc/srsran/ue.conf.example \
    && rm -rf /opt/srsRAN_4G/build
